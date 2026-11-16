/*
 * XIAO ESP32-C6: battery monitor, discharge cutoff, and website-controlled throttle
 *
 * Authority: the board makes every safety decision.
 *   - The website can request ON/OFF and a PWM duty. It cannot clear a latch.
 *   - A critical latch overrides every request until a power cycle.
 *   - If commands stop arriving (page closed, Wi-Fi lost), the load turns off.
 *
 * Reset: power-cycle the XIAO. The latch is RAM-only, and the boot check refuses
 * to arm unless the rested pack voltage is above V_REARM.
 *
 * Wiring (unchanged):
 *   INA219 on D4 (SDA) and D5 (SCL), address 0x40, shunt in the P+ path.
 *   NTC divider: 3V3 -- 10k -- midpoint -- NTC -- GND, midpoint to A1 (GPIO1).
 *   Load: 56 ohm from node N (after the shunt) to IRLZ44N drain. Source to P-.
 *   IRLZ44N gate on D2 (GPIO2), 10k gate-to-source pulldown.
 *
 * Current sign: positive = discharge.
 *
 * Libraries: ArduinoJson (install from Library Manager). Core: arduino-esp32 3.x.
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <math.h>

// ==================== NETWORK ====================
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// Shared base path. The dashboard uses the same base.
const char* FIREBASE_BASE = "https://YOUR-PROJECT-default-rtdb.firebaseio.com/bms";

const unsigned long PUSH_INTERVAL_MS = 2000;
const unsigned long CMD_POLL_MS      = 1000;
const unsigned long CMD_TIMEOUT_MS   = 6000;   // no fresh command for this long = load off
const unsigned long WIFI_RETRY_MS    = 10000;

// ==================== PINS & PWM ====================
const int PIN_GATE = D2;   // IRLZ44N gate (GPIO2)
const int PIN_NTC  = A1;   // NTC divider midpoint (GPIO1)
const uint32_t PWM_HZ = 1000;

// ==================== INA219 ====================
const uint8_t INA219_ADDR = 0x40;
const uint8_t REG_CONFIG      = 0x00;
const uint8_t REG_SHUNT_VOLT  = 0x01;
const uint8_t REG_BUS_VOLT    = 0x02;
const uint8_t REG_CURRENT     = 0x04;
const uint8_t REG_CALIBRATION = 0x05;

// ==================== THERMISTOR ====================
const float SERIES_RESISTOR    = 10000.0;
const float NOMINAL_RESISTANCE = 10000.0;
const float NOMINAL_TEMP       = 25.0;
const float B_COEFFICIENT      = 3950.0;
const float SUPPLY_MV          = 3300.0;

// ==================== SAFETY THRESHOLDS (1S cell) ====================
const float V_WARN   = 3.50;
const float V_CUTOFF = 3.30;
const float V_REARM  = 3.60;
const float T_WARN_C = 45.0;
const float T_CRIT_C = 60.0;
const float I_MAX_MA = 2500.0;
const int   INA_FAIL_LIMIT = 5;
const int   CRIT_DEBOUNCE  = 3;   // consecutive samples before voltage/temp latch

// ==================== STATE ====================
enum BmsState { ST_NORMAL, ST_WARNING, ST_OFF, ST_CRITICAL };
BmsState state = ST_OFF;
BmsState lastPrinted = ST_OFF;

bool latched = false;               // RAM only
float busV = 0.0, shuntMv = 0.0, currentMa = 0.0;
float tempC = NAN;
bool inaOk = false, ntcOk = false;
int inaFailCount = 0;
int vCritCount = 0, tCritCount = 0;

// ==================== COMMAND FROM WEBSITE ====================
// Written by cmdTask, read by the control loop. Single 32-bit values, so reads are not torn.
volatile bool cmdEnabled = false;
volatile int  cmdDuty = 0;
volatile uint32_t lastCmdSeq = 0;
volatile unsigned long lastCmdChangeMs = 0;
int appliedDuty = 0;

// ==================== TELEMETRY ====================
struct Telemetry {
  char state[12];
  bool latched, inaOk, ntcOk, cmdEnabled;
  float busV, currentMa, shuntMv, tempC;
  int dutyPct;
  unsigned long uptimeMs;
};
QueueHandle_t telemetryQ;

WiFiClientSecure pushClient;   // one TLS client per task, never shared
WiFiClientSecure cmdClient;

String liveUrl, cmdUrl;

// ==================== INA219 ====================
void ina219Write(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF);
  Wire.write(value & 0xFF);
  Wire.endTransmission();
}

int16_t ina219Read(uint8_t reg, bool& ok) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) { ok = false; return 0; }
  if (Wire.requestFrom(INA219_ADDR, (uint8_t)2) != 2) { ok = false; return 0; }
  uint16_t hi = Wire.read();
  uint16_t lo = Wire.read();
  ok = true;
  return (int16_t)((hi << 8) | lo);
}

void readINA219() {
  bool a = false, b = false, c = false;
  uint16_t rawBus   = (uint16_t)ina219Read(REG_BUS_VOLT, a);
  int16_t  rawShunt = ina219Read(REG_SHUNT_VOLT, b);
  int16_t  rawCur   = ina219Read(REG_CURRENT, c);

  inaOk = a && b && c;
  if (!inaOk) return;

  busV      = ((rawBus >> 3) * 4) / 1000.0;
  shuntMv   = rawShunt * 0.01;
  currentMa = rawCur * 0.1;
}

// ==================== THERMISTOR ====================
void readNTC() {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogReadMilliVolts(PIN_NTC);
    delay(2);
  }
  float vMv = sum / 16.0;

  if (vMv <= 50.0 || vMv >= SUPPLY_MV - 50.0) {
    ntcOk = false;
    tempC = NAN;
    return;
  }

  float rNtc = SERIES_RESISTOR * vMv / (SUPPLY_MV - vMv);
  float invT = 1.0 / (NOMINAL_TEMP + 273.15)
             + log(rNtc / NOMINAL_RESISTANCE) / B_COEFFICIENT;
  tempC = 1.0 / invT - 273.15;
  ntcOk = true;
}

// ==================== COMMAND LOGIC ====================
bool commandFresh() {
  return (millis() - lastCmdChangeMs) <= CMD_TIMEOUT_MS;
}

bool commandAllowed() {
  return !latched && cmdEnabled && commandFresh();
}

void applyGate() {
  int pct = commandAllowed() ? (int)cmdDuty : 0;
  appliedDuty = constrain(pct, 0, 100);
  ledcWrite(PIN_GATE, appliedDuty * 255 / 100);
}

// Runs at lower priority. Fetches the website's command once a second.
void fetchCommand() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(cmdClient, cmdUrl);
  http.setTimeout(3000);
  int code = http.GET();

  if (code == 200) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream());
    if (!err) {
      bool en = doc["enabled"] | false;
      int duty = doc["duty"] | 0;
      uint32_t seq = doc["seq"] | 0;

      cmdEnabled = en;
      cmdDuty = constrain(duty, 0, 100);
      if (seq != lastCmdSeq) {
        lastCmdSeq = seq;
        lastCmdChangeMs = millis();
      }
    }
  }
  http.end();
}

void cmdTask(void* arg) {
  for (;;) {
    fetchCommand();
    vTaskDelay(pdMS_TO_TICKS(CMD_POLL_MS));
  }
}

// ==================== SAFETY LOGIC ====================
const char* stateName() {
  switch (state) {
    case ST_NORMAL:   return "NORMAL";
    case ST_WARNING:  return "WARNING";
    case ST_OFF:      return "OFF";
    default:          return "CRITICAL";
  }
}

void evaluate() {
  // Voltage and temperature must stay bad for CRIT_DEBOUNCE samples. Impossible
  // voltages are ignored rather than treated as real.
  bool vBad = inaOk && busV <= V_CUTOFF && busV >= 2.5;
  bool tBad = ntcOk && tempC >= T_CRIT_C;
  bool iBad = inaOk && currentMa > I_MAX_MA && currentMa < 10000;

  vCritCount = vBad ? vCritCount + 1 : 0;
  tCritCount = tBad ? tCritCount + 1 : 0;

  if (!latched) {
    if (vCritCount >= CRIT_DEBOUNCE)      { latched = true; Serial.println("LATCH: voltage"); }
    else if (tCritCount >= CRIT_DEBOUNCE) { latched = true; Serial.println("LATCH: temperature"); }
    else if (iBad)                        { latched = true; Serial.println("LATCH: overcurrent"); }
  }

  bool vWarn = inaOk && busV <= V_WARN;
  bool tWarn = ntcOk && tempC >= T_WARN_C;
  bool sensorFault = !inaOk || !ntcOk;

  if (latched)                                state = ST_CRITICAL;
  else if (vWarn || tWarn || sensorFault)     state = ST_WARNING;
  else if (!commandAllowed())                 state = ST_OFF;
  else                                        state = ST_NORMAL;

  applyGate();

  if (state != lastPrinted) {
    Serial.printf("STATE -> %s\n", stateName());
    lastPrinted = state;
  }
}

// ==================== TELEMETRY ====================
Telemetry buildTelemetry(unsigned long now) {
  Telemetry t;
  strncpy(t.state, stateName(), sizeof(t.state) - 1);
  t.state[sizeof(t.state) - 1] = '\0';
  t.latched    = latched;
  t.inaOk      = inaOk;
  t.ntcOk      = ntcOk;
  t.cmdEnabled = cmdEnabled;
  t.busV       = busV;
  t.currentMa  = currentMa;
  t.shuntMv    = shuntMv;
  t.tempC      = tempC;
  t.dutyPct    = appliedDuty;
  t.uptimeMs   = now;
  return t;
}

void pushToFirebase(const Telemetry& t) {
  if (WiFi.status() != WL_CONNECTED) return;

  String json = "{";
  json += "\"state\":\"";     json += t.state;                                   json += "\",";
  json += "\"latched\":";     json += (t.latched ? "true" : "false");            json += ",";
  json += "\"inaOk\":";       json += (t.inaOk ? "true" : "false");              json += ",";
  json += "\"ntcOk\":";       json += (t.ntcOk ? "true" : "false");              json += ",";
  json += "\"cmdEnabled\":";  json += (t.cmdEnabled ? "true" : "false");         json += ",";
  json += "\"dutyPct\":";     json += String(t.dutyPct);                         json += ",";
  json += "\"busV\":";        json += String(t.busV, 3);                         json += ",";
  json += "\"currentMa\":";   json += String(t.currentMa, 1);                    json += ",";
  json += "\"shuntMv\":";     json += String(t.shuntMv, 2);                      json += ",";
  json += "\"tempC\":";       json += (t.ntcOk ? String(t.tempC, 1) : String("null")); json += ",";
  json += "\"uptimeMs\":";    json += String(t.uptimeMs);
  json += "}";

  HTTPClient http;
  http.begin(pushClient, liveUrl);
  http.setTimeout(4000);
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(json);
  if (code != 200) Serial.printf("Firebase PUT returned %d\n", code);
  http.end();
}

void pushTask(void* arg) {
  Telemetry t;
  for (;;) {
    if (xQueueReceive(telemetryQ, &t, portMAX_DELAY) == pdTRUE) {
      pushToFirebase(t);
      vTaskDelay(pdMS_TO_TICKS(PUSH_INTERVAL_MS));
    }
  }
}

// ==================== SETUP ====================
void setup() {
  // PWM on the gate, starting at 0 duty (load off).
  ledcAttach(PIN_GATE, PWM_HZ, 8);
  ledcWrite(PIN_GATE, 0);

  Serial.begin(115200);
  delay(200);   // no "wait for Serial": the board must run on battery with no USB host

  Wire.begin(D4, D5);
  ina219Write(REG_CALIBRATION, 0x1000);
  ina219Write(REG_CONFIG, 0x399F);
  delay(100);

  readINA219();
  if (!inaOk || busV < V_REARM) {
    latched = true;
    Serial.println("BOOT: rested voltage below V_REARM or no INA219, staying latched OFF");
  }
  readNTC();
  evaluate();

  liveUrl = String(FIREBASE_BASE) + "/live.json";
  cmdUrl  = String(FIREBASE_BASE) + "/cmd.json";
  telemetryQ = xQueueCreate(1, sizeof(Telemetry));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  pushClient.setInsecure();   // demo only: skips TLS certificate verification
  cmdClient.setInsecure();

  xTaskCreate(pushTask, "push", 8192, NULL, 0, NULL);
  xTaskCreate(cmdTask,  "cmd",  8192, NULL, 0, NULL);
}

// ==================== LOOP ====================
void loop() {
  static unsigned long tFast = 0, tNtc = 0, tWifi = 0;
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED && now - tWifi >= WIFI_RETRY_MS) {
    tWifi = now;
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  if (now - tNtc >= 2000) {
    tNtc = now;
    readNTC();
  }

  if (now - tFast >= 1000) {
    tFast = now;

    readINA219();
    inaFailCount = inaOk ? 0 : inaFailCount + 1;
    if (inaFailCount >= INA_FAIL_LIMIT) latched = true;

    evaluate();

    Telemetry t = buildTelemetry(now);
    xQueueOverwrite(telemetryQ, &t);

    Serial.printf("V=%.3f I=%.1fmA T=%.1fC duty=%d%% state=%s latched=%d\n",
                  busV, currentMa, ntcOk ? tempC : -999.0, appliedDuty, stateName(), latched);
  }
}
