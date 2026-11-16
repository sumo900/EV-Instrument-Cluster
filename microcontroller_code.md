#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>

// ==================== WI-FI CREDENTIALS ====================
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

WebServer server(80);

// ==================== INA219 SETUP ====================
const uint8_t INA219_ADDR = 0x40;

const uint8_t REG_CONFIG      = 0x00;
const uint8_t REG_SHUNT_VOLT  = 0x01;
const uint8_t REG_BUS_VOLT    = 0x02;
const uint8_t REG_POWER       = 0x03;
const uint8_t REG_CURRENT     = 0x04;
const uint8_t REG_CALIBRATION = 0x05;

// ==================== THERMISTOR SETUP ====================
const int thermistorPin = A1;              // Using A1 (GPIO 1) on XIAO ESP32C6
const float SERIES_RESISTOR = 10000.0;     // 10k ohm fixed resistor
const float NOMINAL_RESISTANCE = 10000.0;  // 10k ohm at 25°C
const float NOMINAL_TEMP = 25.0;           // 25°C
const float B_COEFFICIENT = 3950.0;        // B-value

// ==================== GLOBAL SENSOR VARIABLES ====================
float busVoltage_V = 0.0;
float shuntVoltage_mV = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
float resistance = 0.0;
float tempCelsius = 0.0;

// ==================== TIMING VARIABLES ====================
unsigned long previousMillis = 0;
const unsigned long interval = 1000;       // 1-second base tick
int secondCounter = 0;

void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); 
  Wire.write(value & 0xFF);        
  Wire.endTransmission();
}

int16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte  = Wire.read();
    return (int16_t)((highByte << 8) | lowByte);
  }
  return 0;
}

void readINA219() {
  uint16_t rawBus = (uint16_t)readRegister(REG_BUS_VOLT);
  busVoltage_V = ((rawBus >> 3) * 4) / 1000.0;

  int16_t rawShunt = readRegister(REG_SHUNT_VOLT);
  shuntVoltage_mV = rawShunt * 0.01;

  int16_t rawCurrent = readRegister(REG_CURRENT);
  current_mA = rawCurrent * 0.1;

  power_mW = busVoltage_V * current_mA;
}

void readThermistor() {
  float totalMilliVolts = 0;
  for (int i = 0; i < 10; i++) {
    totalMilliVolts += analogReadMilliVolts(thermistorPin);
    delay(2); 
  }
  float vOut = (totalMilliVolts / 10.0) / 1000.0; 
  float vIn = 3.3; 

  resistance = SERIES_RESISTOR * (vOut / (vIn - vOut));

  float steinhart = resistance / NOMINAL_RESISTANCE;
  steinhart = log(steinhart);
  steinhart /= B_COEFFICIENT;
  steinhart += 1.0 / (NOMINAL_TEMP + 273.15);
  steinhart = 1.0 / steinhart;
  tempCelsius = steinhart - 273.15;
}

// HTML Web Page Handler
void handleRoot() {
  String html = "<html lang=\"en\"><head><meta charset=\"UTF-8\">";
  html += "<meta http-equiv=\"refresh\" content=\"3\">"; // Auto refresh every 3 seconds
  html += "<title>XIAO Battery Monitor</title>";
  html += "<style>";
  html += "body { font-family: Arial, sans-serif; background: #121212; color: #e0e0e0; text-align: center; padding-top: 30px; }";
  html += ".card { background: #1e1e1e; padding: 20px; margin: 15px auto; border-radius: 10px; width: 320px; box-shadow: 0 4px 8px rgba(0,0,0,0.4); }";
  html += "h2 { color: #4CAF50; margin-top: 0; }";
  html += "p { font-size: 16px; margin: 8px 0; }";
  html += "</style></head><body>";
  
  html += "<h1>XIAO ESP32C6 Monitor</h1>";
  
  html += "<div class=\"card\">";
  html += "<h2>INA219 Battery</h2>";
  html += "<p><b>Bus Voltage:</b> " + String(busVoltage_V, 3) + " V</p>";
  html += "<p><b>Shunt Voltage:</b> " + String(shuntVoltage_mV, 2) + " mV</p>";
  html += "<p><b>Current:</b> " + String(current_mA, 2) + " mA</p>";
  html += "<p><b>Power:</b> " + String(power_mW, 2) + " mW</p>";
  html += "</div>";

  html += "<div class=\"card\">";
  html += "<h2>Battery Temperature</h2>";
  html += "<p><b>Resistance:</b> " + String(resistance, 1) + " &Omega;</p>";
  html += "<p><b>Temperature:</b> " + String(tempCelsius, 2) + " &deg;C</p>";
  html += "</div>";

  html += "</body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Wire.begin(D4, D5);

  // Initialize INA219
  writeRegister(REG_CALIBRATION, 0x1000);
  writeRegister(REG_CONFIG, 0x399F);

  // Connect to Local Wi-Fi (Station Mode)
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  // Start web server routes
  server.on("/", handleRoot);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient(); // Process incoming web browser connections

  unsigned long currentMillis = millis();

  // Non-blocking tick every 1 second
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    secondCounter++;

    // Read power metrics every 1 second
    readINA219();

    // Read temperature every 2 seconds
    if (secondCounter >= 2) {
      readThermistor();
      secondCounter = 0;
    }
  }
}


On Sun, 15 Nov, 2026, 00:04 Kala Kutta, <parlegisdabest@gmail.com> wrote:
#include <Wire.h>
#include <math.h>

// ==================== INA219 SETUP ====================
const uint8_t INA219_ADDR = 0x40;

const uint8_t REG_CONFIG = 0x00;
const uint8_t REG_SHUNT_VOLT = 0x01;
const uint8_t REG_BUS_VOLT = 0x02;
const uint8_t REG_POWER = 0x03;
const uint8_t REG_CURRENT = 0x04;
const uint8_t REG_CALIBRATION = 0x05;

// ==================== THERMISTOR SETUP ====================
const int thermistorPin = A1; // Using A1 (GPIO 1) on XIAO ESP32C6
const float SERIES_RESISTOR = 10000.0; // 10k ohm fixed resistor
const float NOMINAL_RESISTANCE = 10000.0; // 10k ohm at 25°C
const float NOMINAL_TEMP = 25.0; // 25°C
const float B_COEFFICIENT = 3950.0; // B-value

// ==================== TIMING VARIABLES ====================
unsigned long lastINA219Time = 0;
unsigned long lastThermistorTime = 0;
const unsigned long ina219Interval = 1000; // Read INA219 every 1 second
const unsigned long thermistorInterval = 2000; // Read Thermistor every 2 seconds

void writeRegister(uint8_t reg, uint16_t value) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((value >> 8) & 0xFF); // High byte
  Wire.write(value & 0xFF); // Low byte
  Wire.endTransmission();
}

int16_t readRegister(uint8_t reg) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(INA219_ADDR, (uint8_t)2);
  if (Wire.available() == 2) {
    uint16_t highByte = Wire.read();
    uint16_t lowByte = Wire.read();
    return (int16_t)((highByte << 8) | lowByte);
  }
  return 0;
}

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open (optional)

  // Initialize I2C for XIAO ESP32C6 (D4 = SDA, D5 = SCL)
  Wire.begin(D4, D5);

  // Calibrate INA219 for 32V, 2A max with a 0.1 ohm shunt resistor
  writeRegister(REG_CALIBRATION, 0x1000);
  writeRegister(REG_CONFIG, 0x399F);
}

void loop() {
  unsigned long currentMillis = millis();

  // --- TASK 1: Read INA219 (Every 1 second) ---
  if (currentMillis - lastINA219Time >= ina219Interval) {
    lastINA219Time = currentMillis;

    uint16_t rawBus = (uint16_t)readRegister(REG_BUS_VOLT);
    float busVoltage_V = ((rawBus >> 3) * 4) / 1000.0;

    int16_t rawShunt = readRegister(REG_SHUNT_VOLT);
    float shuntVoltage_mV = rawShunt * 0.01;

    int16_t rawCurrent = readRegister(REG_CURRENT);
    float current_mA = rawCurrent * 0.1;

    float power_mW = busVoltage_V * current_mA;

    Serial.println("--- Power Monitor (INA219) ---");
    Serial.print("Bus Voltage: "); Serial.print(busVoltage_V, 3); Serial.println(" V");
    Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage_mV, 2); Serial.println(" mV");
    Serial.print("Current: "); Serial.print(current_mA, 2); Serial.println(" mA");
    Serial.print("Power: "); Serial.print(power_mW, 2); Serial.println(" mW");
  }

  // --- TASK 2: Read Thermistor (Every 2 seconds) ---
  if (currentMillis - lastThermistorTime >= thermistorInterval) {
    lastThermistorTime = currentMillis;

    float totalMilliVolts = 0;
    for (int i = 0; i < 10; i++) {
      totalMilliVolts += analogReadMilliVolts(thermistorPin);
      delay(5); // Small delay for sampling averaging
    }
    float vOut = (totalMilliVolts / 10.0) / 1000.0; 
    float vIn = 3.3; 

    float resistance = SERIES_RESISTOR * (vOut / (vIn - vOut));

    float steinhart = resistance / NOMINAL_RESISTANCE;
    steinhart = log(steinhart);
    steinhart /= B_COEFFICIENT;
    steinhart += 1.0 / (NOMINAL_TEMP + 273.15);
    steinhart = 1.0 / steinhart;
    float tempCelsius = steinhart - 273.15;

    Serial.println("--- Temperature (Thermistor) ---");
    Serial.print("Resistance: ");
    Serial.print(resistance);
    Serial.print(" Ω | Temp: ");
    Serial.print(tempCelsius, 2);
    Serial.println(" °C");
    Serial.println("--------------------------------");
  }
}