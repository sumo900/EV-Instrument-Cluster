/*
 * XIAO ESP32-C6: battery monitor with local discharge cutoff
 *
 * Authority: this board makes every safety decision. The website only displays.
 *
 * Reset: power-cycle the XIAO. The latch is kept in RAM, so a power cycle clears it.
 * The boot check then refuses to arm unless the rested pack voltage is above V_REARM.
 *
 * Wiring (from the hand-drawn diagram, verify with a meter):
 *   INA219 on D4 (SDA) and D5 (SCL), address 0x40.
 *   Shunt in the P+ path: VIN+ at P+, VIN- at XIAO BAT+ (node N).
 *   NTC divider: 3V3 -- 10k -- midpoint -- NTC -- GND, midpoint to A1 (GPIO1).
 *   Load: 56 ohm from node N to IRLZ44N drain. Source to P-.
 *   IRLZ44N gate on D2 (GPIO2), 10k gate-to-source pulldown.
 *
 * Current sign: positive = discharge.
 * Bus voltage is measured after the shunt. Under load the cell is slightly higher
 * than the bus reading by the shunt drop (about 0.19 V at 1.85 A on the 2 ohm load).
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <math.h>

// ==================== NETWORK ====================
const char* WIFI_SSID     = "YOUR_SSID";
const char* WIFI_PASSWORD = "YOUR_PASSWORD";

// Firebase Realtime Database REST endpoint. Replace with your project URL.
const char* FIREBASE_URL  = "https://YOUR-PROJECT-default-rtdb.firebaseio.com/bms/live.json";

const unsigned long PUSH_INTERVAL_MS = 2000;   // telemetry push period
const unsigned long WIFI_RETRY_MS    = 10000;  // reconnect attempt period

// ==================== PINS ====================
const int PIN_GATE = D2;   // IRLZ44N gate (GPIO2)
const int PIN_NTC  = A1;   // NTC divider midpoint (GPIO1)

// ==================== INA219 ====================
const uint8_t INA219_ADDR = 0x40;
const uint8_t REG_CONFIG      = 0x00;
const uint8_t REG_SHUNT_VOLT  = 0x01;
const uint8_t REG_BUS_VOLT    = 0x02;
const uint8_t REG_CURRENT     = 0x04;
const uint8_t REG_CALIBRATION = 0x05;

// ==================== THERMISTOR ====================
const float SERIES_RESISTOR    = 10000.0;  // fixed resistor, 3V3 side
const float NOMINAL_RESISTANCE = 10000.0;  // NTC at 25 C
const float NOMINAL_TEMP       = 25.0;     // C
const float B_COEFFICIENT      = 3950.0;
const float SUPPLY_MV          = 3300.0;   // divider supply, measured on the 3V3 rail

// ==================== SAFETY THRESHOLDS (1S cell) ====================
// Check every value against the cell datasheet before the demo.
const float V_WARN   = 3.50;    // low battery warning
const float V_CUTOFF = 3.30;    // critical: gate off, latched
const float V_REARM  = 3.60;    // boot check: rested voltage needed to arm
const float T_WARN_C = 45.0;    // temperature warning
const float T_CRIT_C = 60.0;    // temperature critical: gate off, latched
const float I_MAX_MA = 2500.0;  // overcurrent critical (discharge positive)
const int   INA_FAIL_LIMIT = 5; // consecutive failed INA219 reads before fail-safe latch

// ==================== STATE ====================
enum BmsState { ST_NORMAL, ST_WARNING, ST_CRITICAL };
BmsState state = ST_NORMAL;
bool latched = false;               // RAM only. Cleared by power cycle plus boot check.

float busV = 0.0, shuntMv = 0.0, currentMa = 0.0;
float tempC = NAN;
bool inaOk = false, ntcOk = false;
int inaFailCount = 0;
BmsState lastPrinted = ST_NORMAL;

// Telemetry handed from the control loop to the network task (queue of length 1).
struct Telemetry {
  char state[12];
  bool latched, inaOk, ntcOk;
  float busV, currentMa, shuntMv, tempC;
  unsigned long uptimeMs;
};
QueueHandle_t telemetryQ;

WiFiClientSecure secureClient;

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

  busV      = ((rawBus >> 3) * 4) / 1000.0;  // bus LSB 4 mV
  shuntMv   = rawShunt * 0.01;               // shunt LSB 10 uV
  currentMa = rawCur * 0.1;                  // current LSB 0.1 mA (cal 0x1000, 0.1 ohm)
}

// ==================== THERMISTOR ====================
void readNTC() {
  long sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogReadMilliVolts(PIN_NTC);
    delay(2);
  }
  float vMv = sum / 16.0;

  // Open or shorted sensor: divider output at a rail. Report fault, do not cut.
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

// ==================== SAFETY LOGIC ====================
void setGate(bool on) {
  digitalWrite(PIN_GATE, on ? HIGH : LOW);
}

void evaluate() {
  // Critical conditions latch the gate off until a power cycle.
  bool vCrit = inaOk && busV <= V_CUTOFF;
  bool iCrit = inaOk && currentMa > I_MAX_MA;
  bool tCrit = ntcOk && tempC >= T_CRIT_C;
  if (vCrit || iCrit || tCrit) latched = true;

  // Warnings do not latch and do not switch the gate.
  bool vWarn = inaOk && busV <= V_WARN;
  bool tWarn = ntcOk && tempC >= T_WARN_C;
  bool sensorFault = !inaOk || !ntcOk;

  if (latched)                          state = ST_CRITICAL;
  else if (vWarn || tWarn || sensorFault) state = ST_WARNING;
  else                                  state = ST_NORMAL;

  setGate(!latched);

  if (state != lastPrinted) {
    Serial.printf("STATE -> %s\n", stateName());
    lastPrinted = state;
  }
}

const char* stateName() {
  switch (state) {
    case ST_NORMAL:  return "NORMAL";
    case ST_WARNING: return "WARNING";
    default:         return "CRITICAL";
  }
}

// ==================== TELEMETRY ====================
Telemetry buildTelemetry(unsigned long now) {
  Telemetry t;
  strncpy(t.state, stateName(), sizeof(t.state) - 1);
  t.state[sizeof(t.state) - 1] = '\0';
  t.latched  = latched;
  t.inaOk    = inaOk;
  t.ntcOk    = ntcOk;
  t.busV     = busV;
  t.currentMa = currentMa;
  t.shuntMv  = shuntMv;
  t.tempC    = tempC;
  t.uptimeMs = now;
  return t;
}

void pushToFirebase(const Telemetry& t) {
  if (WiFi.status() != WL_CONNECTED) return;

  String json = "{";
  json += "\"state\":\"";     json += t.state;                          json += "\",";
  json += "\"latched\":";     json += (t.latched ? "true" : "false");   json += ",";
  json += "\"inaOk\":";       json += (t.inaOk ? "true" : "false");     json += ",";
  json += "\"ntcOk\":";       json += (t.ntcOk ? "true" : "false");     json += ",";
  json += "\"busV\":";        json += String(t.busV, 3);                json += ",";
  json += "\"currentMa\":";   json += String(t.currentMa, 1);           json += ",";
  json += "\"shuntMv\":";     json += String(t.shuntMv, 2);             json += ",";
  json += "\"tempC\":";       json += (t.ntcOk ? String(t.tempC, 1) : String("null")); json += ",";
  json += "\"uptimeMs\":";    json += String(t.uptimeMs);
  json += "}";

  HTTPClient http;
  http.begin(secureClient, FIREBASE_URL);
  http.setTimeout(4000);
  http.addHeader("Content-Type", "application/json");
  int code = http.PUT(json);
  if (code != 200) Serial.printf("Firebase PUT returned %d\n", code);
  http.end();
}

// Runs at lower priority than the control loop, so a slow network call
// can never delay the cutoff decision.
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
  // Gate off first, before anything else. The 10k pulldown also holds it off during reset.
  pinMode(PIN_GATE, OUTPUT);
  digitalWrite(PIN_GATE, LOW);

  Serial.begin(115200);
  delay(200);   // no "wait for Serial": the board must run on battery with no USB host

  Wire.begin(D4, D5);
  ina219Write(REG_CALIBRATION, 0x1000);
  ina219Write(REG_CONFIG, 0x399F);
  delay(100);

  // Boot check: gate is off, so the bus reading is the rested cell voltage.
  readINA219();
  if (!inaOk || busV < V_REARM) {
    latched = true;
    Serial.println("BOOT: rested voltage below V_REARM or no INA219, staying latched OFF");
  }
  readNTC();
  evaluate();

  telemetryQ = xQueueCreate(1, sizeof(Telemetry));

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  secureClient.setInsecure();   // demo only: skips TLS certificate verification

  xTaskCreate(pushTask, "push", 8192, NULL, 0, NULL);
}

// ==================== LOOP ====================
void loop() {
  static unsigned long tFast = 0, tNtc = 0, tWifi = 0;
  unsigned long now = millis();

  // Non-blocking Wi-Fi retry. Never gates the control loop.
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

    // Losing the current sensor means we cannot see the cutoff condition.
    // Fail safe: latch off.
    if (inaFailCount >= INA_FAIL_LIMIT) latched = true;

    evaluate();

    Telemetry t = buildTelemetry(now);
    xQueueOverwrite(telemetryQ, &t);

    Serial.printf("V=%.3f I=%.1fmA T=%.1fC state=%s latched=%d\n",
                  busV, currentMa, ntcOk ? tempC : -999.0, stateName(), latched);
  }
}
