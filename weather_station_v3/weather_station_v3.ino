/**
 * WEATHER STATION V3 - 96-CYCLE DAILY TRACKING WITH SPIFFS BACKUP
 *
 * Version: 3.0
 * Date: January 2026
 *
 * Sensor Addresses:
 * - Wind (RK120-01):        0x01
 * - Rain (RK400-04):        0x02
 * - Solar (RK200-03):       0x03
 * - Atmospheric (RK330-01): 0x05
 *
 * Timing (Testing Mode - seconds):
 * - Temperature: Read every 5 sec, send Min/Max every 15 sec
 * - All other sensors: Read & send every 15 sec (1 cycle)
 * - 96 cycles = 1 simulated day (24 min in testing, 24 hrs in deployment)
 * - Daily summary printed after 96 cycles
 *
 * SWB Monitoring Timeframe: Every 15 minutes (1 cycle in deployment)
 *
 * SPIFFS Backup:
 * - Cycle data saved to SPIFFS every cycle
 * - Daily accumulators saved to SPIFFS every cycle
 * - On power restore, reads last saved state and continues
 *
 * Rain Gauge Clear: Rika Protocol 6.3 (Reg 0x0008, FC 0x06)
 */

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>

// ============== RS485 Configuration ==============
#define MAX485_DE_RE 4
HardwareSerial RS485Serial(2);
#define RX_PIN 16
#define TX_PIN 17
#define BAUD_RATE 9600

// ============== Sensor Addresses ==============
#define WIND_ADDR 0x01
#define RAIN_ADDR 0x02
#define SOLAR_ADDR 0x03
#define ATMOS_ADDR 0x05

// ============== Enable/Disable Sensors ==============
#define HAS_WIND true
#define HAS_RAIN true
#define HAS_SOLAR true
#define HAS_ATMOS true

// ============== Timing Configuration ==============
// TESTING MODE: Using seconds (change to minutes for deployment)
#define TEMP_READ_INTERVAL 5000      // Read temperature every 5 seconds
#define SEND_INTERVAL 15000          // 1 cycle = 15 seconds (deployment: 15 minutes)
#define CYCLES_PER_DAY 96            // 96 cycles = 1 simulated day
#define SUNSHINE_THRESHOLD 120.0     // W/m² (WMO standard)

// SPIFFS Configuration
#define SPIFFS_DAILY_FILE "/daily_data.json"
#define SPIFFS_CYCLE_FILE "/cycle_backup.json"
#define SPIFFS_SAVE_INTERVAL 1       // Save to SPIFFS every N cycles

// ============== Timing Variables ==============
unsigned long lastTempRead = 0;
unsigned long lastDataSend = 0;
unsigned long lastSunshineCheck = 0;

// ============== Current Sensor Readings ==============
float currentTemp = 0;
float currentHumidity = 0;
float currentPressure = 0;
float windSpeed = 0;
float windDir = 0;
float rainfallTotal = 0;
float rainfallPrevious = -1;
float rainfallCycle = 0;
float solarRadiation = 0;

// ============== Per-Cycle Temperature Tracking (5-sec reads within 15-sec cycle) ==============
float tempMin = 100.0;
float tempMax = -100.0;
float tempSum = 0;
int tempReadCount = 0;

// ============== Sunshine Duration Tracking ==============
unsigned long sunshineDurationMs = 0;
unsigned long cycleStartSunshine = 0;
bool isSunshining = false;

// ============== Cycle Counter ==============
int cycleNumber = 0;
int dayNumber = 0;

// ============== Daily Accumulators (across 96 cycles) ==============
struct DailyData {
  // Temperature (from all 5-sec readings across the day)
  float dayTempMin;
  float dayTempMax;
  float dayTempSum;
  int   dayTempCount;

  // Humidity
  float dayHumiditySum;
  float dayHumidityMin;
  float dayHumidityMax;
  int   dayHumidityCount;

  // Pressure
  float dayPressureSum;
  int   dayPressureCount;

  // Wind
  float dayWindSpeedSum;
  float dayWindSpeedMax;
  int   dayWindCount;

  // Solar
  float daySolarSum;
  float daySolarMax;
  int   daySolarCount;

  // Sunshine (total minutes for the day)
  float daySunshineMinutes;

  // Rainfall (sum of all cycle deltas)
  float dayRainfallTotal;

  // VPD
  float dayVPDSum;
  float dayVPDMax;
  float dayVPDMin;
  int   dayVPDCount;

  // Tracking
  int   cyclesCompleted;
  int   cyclesWithErrors;
  unsigned long dayStartMs;
};

DailyData daily;

// ============== SPIFFS Status ==============
bool spiffsReady = false;

// ============== Forward Declarations ==============
void initializeDailyData();
void saveDailyToSPIFFS();
void saveCycleBackup();
bool loadDailyFromSPIFFS();
bool loadCycleBackup();
void printDailySummary();

// ============== CRC Calculation ==============
uint16_t calculateCRC(uint8_t *data, uint8_t length) {
  uint16_t crc = 0xFFFF;
  for (uint8_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ============== RS485 Control ==============
void setTX() {
  digitalWrite(MAX485_DE_RE, HIGH);
  delayMicroseconds(100);
}

void setRX() {
  digitalWrite(MAX485_DE_RE, LOW);
  delayMicroseconds(100);
}

void sendRequest(uint8_t addr, uint8_t func, uint16_t reg, uint16_t num) {
  uint8_t req[8];
  req[0] = addr;
  req[1] = func;
  req[2] = (reg >> 8) & 0xFF;
  req[3] = reg & 0xFF;
  req[4] = (num >> 8) & 0xFF;
  req[5] = num & 0xFF;

  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  while (RS485Serial.available()) RS485Serial.read();

  setTX();
  RS485Serial.write(req, 8);
  RS485Serial.flush();
  setRX();
}

// ============== Sensor Reading Functions ==============

bool readWind() {
  if (!HAS_WIND) return false;

  uint8_t resp[20];
  uint8_t idx = 0;

  sendRequest(WIND_ADDR, 0x03, 0x0000, 0x0002);

  unsigned long start = millis();
  while (millis() - start < 500 && idx < 9) {
    if (RS485Serial.available()) resp[idx++] = RS485Serial.read();
  }

  if (idx < 9) return false;

  windSpeed = ((resp[3] << 8) | resp[4]) / 10.0;
  windDir = (resp[5] << 8) | resp[6];

  return true;
}

bool readRain() {
  if (!HAS_RAIN) return false;

  uint8_t resp[20];
  uint8_t idx = 0;

  sendRequest(RAIN_ADDR, 0x03, 0x0000, 0x0001);

  unsigned long start = millis();
  while (millis() - start < 500 && idx < 7) {
    if (RS485Serial.available()) resp[idx++] = RS485Serial.read();
  }

  if (idx < 7) return false;

  uint16_t rainRaw = (resp[3] << 8) | resp[4];
  rainfallTotal = rainRaw * 0.2;

  if (rainfallPrevious < 0) {
    rainfallPrevious = rainfallTotal;
    rainfallCycle = 0;
  } else {
    rainfallCycle = rainfallTotal - rainfallPrevious;
    if (rainfallCycle < 0) {
      rainfallCycle = rainfallTotal;
    }
  }

  return true;
}

void updateRainfallBaseline() {
  rainfallPrevious = rainfallTotal;
}

bool readSolar() {
  if (!HAS_SOLAR) return false;

  uint8_t resp[20];
  uint8_t idx = 0;

  sendRequest(SOLAR_ADDR, 0x03, 0x0000, 0x0001);

  unsigned long start = millis();
  while (millis() - start < 500 && idx < 7) {
    if (RS485Serial.available()) resp[idx++] = RS485Serial.read();
  }

  if (idx < 7) return false;

  solarRadiation = (resp[3] << 8) | resp[4];

  return true;
}

bool readAtmos() {
  if (!HAS_ATMOS) return false;

  uint8_t resp[20];
  uint8_t idx = 0;

  sendRequest(ATMOS_ADDR, 0x03, 0x0000, 0x0003);

  unsigned long start = millis();
  while (millis() - start < 500 && idx < 11) {
    if (RS485Serial.available()) resp[idx++] = RS485Serial.read();
  }

  if (idx < 11) return false;

  uint16_t tempRaw = (resp[3] << 8) | resp[4];

  if (tempRaw >= 0x8000) {
    currentTemp = (int16_t)(tempRaw - 0xFFFF - 1) / 10.0;
  } else {
    currentTemp = tempRaw / 10.0;
  }

  currentHumidity = ((resp[5] << 8) | resp[6]) / 10.0;
  currentPressure = ((resp[7] << 8) | resp[8]) / 10.0;

  return true;
}

// ============== Sunshine Duration Tracking ==============
void updateSunshineDuration() {
  unsigned long now = millis();

  if (lastSunshineCheck == 0) {
    lastSunshineCheck = now;
    isSunshining = (solarRadiation >= SUNSHINE_THRESHOLD);
    return;
  }

  unsigned long elapsed = now - lastSunshineCheck;

  if (now < lastSunshineCheck) {
    elapsed = (0xFFFFFFFF - lastSunshineCheck) + now + 1;
  }

  if (isSunshining) {
    sunshineDurationMs += elapsed;
  }

  lastSunshineCheck = now;
  isSunshining = (solarRadiation >= SUNSHINE_THRESHOLD);
}

float getSunshineDurationMinutes() {
  return sunshineDurationMs / 60000.0;
}

float getCycleSunshineMinutes() {
  return (sunshineDurationMs - cycleStartSunshine) / 60000.0;
}

// ============== VPD Calculation ==============
float calculateVPD(float tempC, float relHumidity) {
  float es = 0.6108 * exp((17.27 * tempC) / (tempC + 237.3));
  float ea = es * (relHumidity / 100.0);
  return es - ea;
}

// ============== Daily Data Management ==============
void initializeDailyData() {
  daily.dayTempMin = 100.0;
  daily.dayTempMax = -100.0;
  daily.dayTempSum = 0;
  daily.dayTempCount = 0;

  daily.dayHumiditySum = 0;
  daily.dayHumidityMin = 100.0;
  daily.dayHumidityMax = 0;
  daily.dayHumidityCount = 0;

  daily.dayPressureSum = 0;
  daily.dayPressureCount = 0;

  daily.dayWindSpeedSum = 0;
  daily.dayWindSpeedMax = 0;
  daily.dayWindCount = 0;

  daily.daySolarSum = 0;
  daily.daySolarMax = 0;
  daily.daySolarCount = 0;

  daily.daySunshineMinutes = 0;

  daily.dayRainfallTotal = 0;

  daily.dayVPDSum = 0;
  daily.dayVPDMax = 0;
  daily.dayVPDMin = 100.0;
  daily.dayVPDCount = 0;

  daily.cyclesCompleted = 0;
  daily.cyclesWithErrors = 0;
  daily.dayStartMs = millis();

  // Reset sunshine tracking for new day
  sunshineDurationMs = 0;
  cycleStartSunshine = 0;
  lastSunshineCheck = millis();
}

void updateDailyAccumulators(float cycleTempMin, float cycleTempMax, float cycleTempAvg,
                              float humidity, float pressure,
                              float wind, float solar, float cycleSunshine,
                              float rain, float vpd, bool hadErrors) {
  // Temperature - track absolute min/max across all 5-sec readings
  if (cycleTempMin < daily.dayTempMin) daily.dayTempMin = cycleTempMin;
  if (cycleTempMax > daily.dayTempMax) daily.dayTempMax = cycleTempMax;
  daily.dayTempSum += cycleTempAvg;
  daily.dayTempCount++;

  // Humidity
  if (!isnan(humidity) && humidity > 0) {
    daily.dayHumiditySum += humidity;
    if (humidity < daily.dayHumidityMin) daily.dayHumidityMin = humidity;
    if (humidity > daily.dayHumidityMax) daily.dayHumidityMax = humidity;
    daily.dayHumidityCount++;
  }

  // Pressure
  if (!isnan(pressure) && pressure > 0) {
    daily.dayPressureSum += pressure;
    daily.dayPressureCount++;
  }

  // Wind
  if (wind >= 0) {
    daily.dayWindSpeedSum += wind;
    if (wind > daily.dayWindSpeedMax) daily.dayWindSpeedMax = wind;
    daily.dayWindCount++;
  }

  // Solar
  if (solar >= 0) {
    daily.daySolarSum += solar;
    if (solar > daily.daySolarMax) daily.daySolarMax = solar;
    daily.daySolarCount++;
  }

  // Sunshine
  daily.daySunshineMinutes += cycleSunshine;

  // Rainfall (accumulate cycle deltas)
  daily.dayRainfallTotal += rain;

  // VPD
  if (vpd > 0) {
    daily.dayVPDSum += vpd;
    if (vpd > daily.dayVPDMax) daily.dayVPDMax = vpd;
    if (vpd < daily.dayVPDMin) daily.dayVPDMin = vpd;
    daily.dayVPDCount++;
  }

  daily.cyclesCompleted++;
  if (hadErrors) daily.cyclesWithErrors++;
}

// ============== SPIFFS Functions ==============
void initSPIFFS() {
  if (SPIFFS.begin(true)) {
    spiffsReady = true;
    Serial.println("[SPIFFS] Mounted successfully");

    // Show available space
    size_t totalBytes = SPIFFS.totalBytes();
    size_t usedBytes = SPIFFS.usedBytes();
    Serial.printf("[SPIFFS] Total: %d bytes | Used: %d bytes | Free: %d bytes\n",
                  totalBytes, usedBytes, totalBytes - usedBytes);
  } else {
    spiffsReady = false;
    Serial.println("[SPIFFS] Mount FAILED - data will NOT be saved");
  }
}

void saveDailyToSPIFFS() {
  if (!spiffsReady) return;

  StaticJsonDocument<1024> doc;

  doc["day"] = dayNumber;
  doc["cycle"] = daily.cyclesCompleted;
  doc["cycleNumber"] = cycleNumber;

  // Temperature
  doc["tMin"] = daily.dayTempMin;
  doc["tMax"] = daily.dayTempMax;
  doc["tSum"] = daily.dayTempSum;
  doc["tCnt"] = daily.dayTempCount;

  // Humidity
  doc["hSum"] = daily.dayHumiditySum;
  doc["hMin"] = daily.dayHumidityMin;
  doc["hMax"] = daily.dayHumidityMax;
  doc["hCnt"] = daily.dayHumidityCount;

  // Pressure
  doc["pSum"] = daily.dayPressureSum;
  doc["pCnt"] = daily.dayPressureCount;

  // Wind
  doc["wSum"] = daily.dayWindSpeedSum;
  doc["wMax"] = daily.dayWindSpeedMax;
  doc["wCnt"] = daily.dayWindCount;

  // Solar
  doc["sSum"] = daily.daySolarSum;
  doc["sMax"] = daily.daySolarMax;
  doc["sCnt"] = daily.daySolarCount;

  // Sunshine & Rain
  doc["sunMin"] = daily.daySunshineMinutes;
  doc["rain"] = daily.dayRainfallTotal;

  // VPD
  doc["vSum"] = daily.dayVPDSum;
  doc["vMax"] = daily.dayVPDMax;
  doc["vMin"] = daily.dayVPDMin;
  doc["vCnt"] = daily.dayVPDCount;

  doc["errors"] = daily.cyclesWithErrors;
  doc["rainPrev"] = rainfallPrevious;
  doc["sunDurMs"] = sunshineDurationMs;

  File file = SPIFFS.open(SPIFFS_DAILY_FILE, FILE_WRITE);
  if (file) {
    serializeJson(doc, file);
    file.close();
    Serial.printf("[SPIFFS] Daily data saved (cycle %d/%d)\n",
                  daily.cyclesCompleted, CYCLES_PER_DAY);
  } else {
    Serial.println("[SPIFFS] ERROR saving daily data");
  }
}

bool loadDailyFromSPIFFS() {
  if (!spiffsReady) return false;

  if (!SPIFFS.exists(SPIFFS_DAILY_FILE)) {
    Serial.println("[SPIFFS] No daily backup found - starting fresh");
    return false;
  }

  File file = SPIFFS.open(SPIFFS_DAILY_FILE, FILE_READ);
  if (!file) return false;

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();

  if (err) {
    Serial.printf("[SPIFFS] Parse error: %s\n", err.c_str());
    return false;
  }

  dayNumber = doc["day"] | 0;
  daily.cyclesCompleted = doc["cycle"] | 0;
  cycleNumber = doc["cycleNumber"] | 0;

  // Temperature
  daily.dayTempMin = doc["tMin"] | 100.0;
  daily.dayTempMax = doc["tMax"] | -100.0;
  daily.dayTempSum = doc["tSum"] | 0.0;
  daily.dayTempCount = doc["tCnt"] | 0;

  // Humidity
  daily.dayHumiditySum = doc["hSum"] | 0.0;
  daily.dayHumidityMin = doc["hMin"] | 100.0;
  daily.dayHumidityMax = doc["hMax"] | 0.0;
  daily.dayHumidityCount = doc["hCnt"] | 0;

  // Pressure
  daily.dayPressureSum = doc["pSum"] | 0.0;
  daily.dayPressureCount = doc["pCnt"] | 0;

  // Wind
  daily.dayWindSpeedSum = doc["wSum"] | 0.0;
  daily.dayWindSpeedMax = doc["wMax"] | 0.0;
  daily.dayWindCount = doc["wCnt"] | 0;

  // Solar
  daily.daySolarSum = doc["sSum"] | 0.0;
  daily.daySolarMax = doc["sMax"] | 0.0;
  daily.daySolarCount = doc["sCnt"] | 0;

  // Sunshine & Rain
  daily.daySunshineMinutes = doc["sunMin"] | 0.0;
  daily.dayRainfallTotal = doc["rain"] | 0.0;

  // VPD
  daily.dayVPDSum = doc["vSum"] | 0.0;
  daily.dayVPDMax = doc["vMax"] | 0.0;
  daily.dayVPDMin = doc["vMin"] | 100.0;
  daily.dayVPDCount = doc["vCnt"] | 0;

  daily.cyclesWithErrors = doc["errors"] | 0;
  rainfallPrevious = doc["rainPrev"] | -1.0;
  sunshineDurationMs = doc["sunDurMs"] | 0;

  daily.dayStartMs = millis();

  Serial.println("[SPIFFS] *** RECOVERED FROM POWER LOSS ***");
  Serial.printf("[SPIFFS] Day %d | Resuming at cycle %d/%d\n",
                dayNumber, daily.cyclesCompleted, CYCLES_PER_DAY);
  Serial.printf("[SPIFFS] Recovered: TempMin=%.1f TempMax=%.1f Rain=%.1f Sunshine=%.2f min\n",
                daily.dayTempMin, daily.dayTempMax,
                daily.dayRainfallTotal, daily.daySunshineMinutes);

  return true;
}

void saveCycleBackup() {
  if (!spiffsReady) return;

  StaticJsonDocument<256> doc;
  doc["temp"] = currentTemp;
  doc["hum"] = currentHumidity;
  doc["pres"] = currentPressure;
  doc["wind"] = windSpeed;
  doc["wdir"] = windDir;
  doc["solar"] = solarRadiation;
  doc["rainT"] = rainfallTotal;
  doc["rainC"] = rainfallCycle;
  doc["cycle"] = cycleNumber;

  File file = SPIFFS.open(SPIFFS_CYCLE_FILE, FILE_WRITE);
  if (file) {
    serializeJson(doc, file);
    file.close();
  }
}

void clearSPIFFSDaily() {
  if (!spiffsReady) return;
  if (SPIFFS.exists(SPIFFS_DAILY_FILE)) {
    SPIFFS.remove(SPIFFS_DAILY_FILE);
  }
  Serial.println("[SPIFFS] Daily data cleared for new day");
}

// ============== Clear Functions ==============

/**
 * Clear Rainfall Sensor (Rika Protocol Section 6.3)
 * Register: 0x0008, FC: 0x06, Value: 0x0000
 * Sensor address: 0x02
 */
void clearRainfallSensor() {
  if (!HAS_RAIN) return;

  Serial.println("[RAIN] Clearing rainfall value (Rika protocol 6.3)...");
  Serial.printf("[RAIN] Sensor address: 0x%02X\n", RAIN_ADDR);

  uint8_t req[8];
  req[0] = RAIN_ADDR;
  req[1] = 0x06;
  req[2] = 0x00;
  req[3] = 0x08;
  req[4] = 0x00;
  req[5] = 0x00;

  uint16_t crc = calculateCRC(req, 6);
  req[6] = crc & 0xFF;
  req[7] = (crc >> 8) & 0xFF;

  Serial.print("[RAIN] TX: ");
  for (int i = 0; i < 8; i++) Serial.printf("%02X ", req[i]);
  Serial.println();

  while (RS485Serial.available()) RS485Serial.read();

  setTX();
  RS485Serial.write(req, 8);
  RS485Serial.flush();
  setRX();

  delay(500);

  uint8_t resp[8];
  int respLen = 0;
  while (RS485Serial.available() && respLen < 8) {
    resp[respLen++] = RS485Serial.read();
  }

  if (respLen > 0) {
    Serial.print("[RAIN] RX: ");
    for (int i = 0; i < respLen; i++) Serial.printf("%02X ", resp[i]);
    Serial.println();
    if (respLen >= 6 && resp[0] == RAIN_ADDR && resp[1] == 0x06) {
      Serial.println("[RAIN] Sensor acknowledged clear command");
    }
  } else {
    Serial.println("[RAIN] No response from sensor");
  }

  delay(300);
  float oldTotal = rainfallTotal;
  if (readRain()) {
    if (rainfallTotal < oldTotal - 0.1) {
      Serial.printf("[RAIN] SUCCESS! Cleared: %.1f -> %.1f mm\n", oldTotal, rainfallTotal);
    } else if (rainfallTotal < 0.5) {
      Serial.printf("[RAIN] SUCCESS! Sensor reads: %.1f mm\n", rainfallTotal);
    } else {
      Serial.printf("[RAIN] Sensor still reads: %.1f mm\n", rainfallTotal);
    }
  }

  rainfallPrevious = rainfallTotal;
  rainfallCycle = 0;
  Serial.println("[RAIN] Software tracking reset");
}

void resetRainfallTracking() {
  Serial.println("[RAIN] Resetting rainfall baseline...");
  rainfallPrevious = rainfallTotal;
  rainfallCycle = 0;
  Serial.printf("[RAIN] Baseline set at %.1f mm\n", rainfallPrevious);
}

void clearSunshineDuration() {
  sunshineDurationMs = 0;
  cycleStartSunshine = 0;
  lastSunshineCheck = millis();
  Serial.println("[SOLAR] Sunshine duration cleared");
}

void resetTempTracking() {
  tempMin = 100.0;
  tempMax = -100.0;
  tempSum = 0;
  tempReadCount = 0;
}

// ============== Daily Summary ==============
void printDailySummary() {
  float tempAvg = (daily.dayTempCount > 0) ? (daily.dayTempSum / daily.dayTempCount) : 0;
  float humAvg = (daily.dayHumidityCount > 0) ? (daily.dayHumiditySum / daily.dayHumidityCount) : 0;
  float presAvg = (daily.dayPressureCount > 0) ? (daily.dayPressureSum / daily.dayPressureCount) : 0;
  float windAvg = (daily.dayWindCount > 0) ? (daily.dayWindSpeedSum / daily.dayWindCount) : 0;
  float solarAvg = (daily.daySolarCount > 0) ? (daily.daySolarSum / daily.daySolarCount) : 0;
  float vpdAvg = (daily.dayVPDCount > 0) ? (daily.dayVPDSum / daily.dayVPDCount) : 0;

  unsigned long dayDuration = (millis() - daily.dayStartMs) / 1000;

  Serial.println("\n");
  Serial.println("################################################################");
  Serial.println("##                                                            ##");
  Serial.printf("##     24-HOUR DAILY SUMMARY - DAY %d                          \n", dayNumber);
  Serial.println("##                                                            ##");
  Serial.println("################################################################");
  Serial.println();

  Serial.println("┌──────────────────────────────────────────────────────────────┐");
  Serial.println("│  TEMPERATURE                                                 │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Minimum:       %.1f C                                 \n", daily.dayTempMin);
  Serial.printf("│  Daily Maximum:       %.1f C                                 \n", daily.dayTempMax);
  Serial.printf("│  Daily Average:       %.1f C                                 \n", tempAvg);
  Serial.printf("│  Readings:            %d                                     \n", daily.dayTempCount);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  RELATIVE HUMIDITY                                           │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Minimum:       %.1f %%RH                              \n", daily.dayHumidityMin);
  Serial.printf("│  Daily Maximum:       %.1f %%RH                              \n", daily.dayHumidityMax);
  Serial.printf("│  Daily Average:       %.1f %%RH                              \n", humAvg);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  ATMOSPHERIC PRESSURE                                        │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Average:       %.1f hPa                               \n", presAvg);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  WIND                                                        │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Average:       %.1f m/s                               \n", windAvg);
  Serial.printf("│  Daily Maximum:       %.1f m/s                               \n", daily.dayWindSpeedMax);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  SOLAR RADIATION                                             │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Average:       %.0f W/m2                              \n", solarAvg);
  Serial.printf("│  Daily Maximum:       %.0f W/m2                              \n", daily.daySolarMax);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  SUNSHINE DURATION                                           │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Total Sunshine:      %.2f min                               \n", daily.daySunshineMinutes);
  Serial.printf("│  Sunshine Hours:      %.2f hrs                               \n", daily.daySunshineMinutes / 60.0);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  RAINFALL                                                    │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Total:         %.1f mm                                \n", daily.dayRainfallTotal);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  VAPOR PRESSURE DEFICIT (VPD)                                │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Daily Average:       %.3f kPa                               \n", vpdAvg);
  Serial.printf("│  Daily Minimum:       %.3f kPa                               \n", daily.dayVPDMin);
  Serial.printf("│  Daily Maximum:       %.3f kPa                               \n", daily.dayVPDMax);
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.println("│  DATA QUALITY                                                │");
  Serial.println("├──────────────────────────────────────────────────────────────┤");
  Serial.printf("│  Cycles Completed:    %d / %d                                \n", daily.cyclesCompleted, CYCLES_PER_DAY);
  Serial.printf("│  Cycles with Errors:  %d                                     \n", daily.cyclesWithErrors);
  Serial.printf("│  Data Completeness:   %.1f%%                                 \n",
                (daily.cyclesCompleted * 100.0) / CYCLES_PER_DAY);
  Serial.printf("│  Actual Duration:     %lu sec                                \n", dayDuration);
  Serial.println("└──────────────────────────────────────────────────────────────┘");

  // CSV summary for easy export
  Serial.println("\n>> DAILY CSV SUMMARY:");
  Serial.printf("DAY,%d,TEMP_MIN,%.1f,TEMP_MAX,%.1f,TEMP_AVG,%.1f,HUM_MIN,%.1f,HUM_MAX,%.1f,HUM_AVG,%.1f,"
                "PRES_AVG,%.1f,WIND_AVG,%.1f,WIND_MAX,%.1f,SOLAR_AVG,%.0f,SOLAR_MAX,%.0f,"
                "SUNSHINE_MIN,%.2f,SUNSHINE_HRS,%.2f,RAIN_MM,%.1f,"
                "VPD_AVG,%.3f,VPD_MIN,%.3f,VPD_MAX,%.3f,"
                "CYCLES,%d,ERRORS,%d\n",
                dayNumber,
                daily.dayTempMin, daily.dayTempMax, tempAvg,
                daily.dayHumidityMin, daily.dayHumidityMax, humAvg,
                presAvg,
                windAvg, daily.dayWindSpeedMax,
                solarAvg, daily.daySolarMax,
                daily.daySunshineMinutes, daily.daySunshineMinutes / 60.0,
                daily.dayRainfallTotal,
                vpdAvg, daily.dayVPDMin, daily.dayVPDMax,
                daily.cyclesCompleted, daily.cyclesWithErrors);

  Serial.println("\n################################################################\n");
}

// ============== Setup ==============
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(MAX485_DE_RE, OUTPUT);
  setRX();
  RS485Serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║     WEATHER STATION V3 - 96-CYCLE DAILY TRACKING            ║");
  Serial.println("║     With SPIFFS Backup & Power Recovery                      ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝");
  Serial.println();
  Serial.println("Sensor Configuration:");
  Serial.printf("  [%c] Wind Sensor RK120-01    (Addr: 0x%02X)\n", HAS_WIND ? 'x' : ' ', WIND_ADDR);
  Serial.printf("  [%c] Rain Gauge RK400-04     (Addr: 0x%02X)\n", HAS_RAIN ? 'x' : ' ', RAIN_ADDR);
  Serial.printf("  [%c] Pyranometer RK200-03    (Addr: 0x%02X)\n", HAS_SOLAR ? 'x' : ' ', SOLAR_ADDR);
  Serial.printf("  [%c] Atmospheric RK330-01    (Addr: 0x%02X)\n", HAS_ATMOS ? 'x' : ' ', ATMOS_ADDR);
  Serial.println();
  Serial.println("Timing (TESTING MODE - seconds):");
  Serial.println("  - Temperature: Read every 5 sec, Min/Max per 15-sec cycle");
  Serial.println("  - All sensors: Read & send every 15 sec (1 cycle)");
  Serial.printf("  - 1 simulated day = %d cycles = %d sec (%d min)\n",
                CYCLES_PER_DAY, CYCLES_PER_DAY * 15, (CYCLES_PER_DAY * 15) / 60);
  Serial.printf("  - Sunshine Threshold: %.0f W/m2 (WMO)\n", SUNSHINE_THRESHOLD);
  Serial.println();
  Serial.println("  NOTE: For deployment, change SEND_INTERVAL to 900000 (15 min)");
  Serial.println("        96 cycles x 15 min = 1440 min = 24 hours");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  c - Clear rainfall sensor (Rika protocol 6.3)");
  Serial.println("  b - Reset rainfall baseline");
  Serial.println("  s - Clear sunshine duration");
  Serial.println("  r - Reset temperature min/max");
  Serial.println("  t - Test all sensors");
  Serial.println("  d - Show current daily accumulators");
  Serial.println("  f - Force daily summary now");
  Serial.println("  x - Clear SPIFFS and start fresh");
  Serial.println();

  // Initialize SPIFFS
  initSPIFFS();

  // Try to recover from power loss
  bool recovered = loadDailyFromSPIFFS();

  if (!recovered) {
    dayNumber = 1;
    cycleNumber = 0;
    initializeDailyData();
    Serial.println("[BOOT] Starting fresh - Day 1, Cycle 0");
  }

  // Initialize timing
  lastTempRead = millis();
  lastDataSend = millis();
  if (lastSunshineCheck == 0) lastSunshineCheck = millis();

  delay(2000);
  Serial.println("\nStarting data collection...");
  Serial.printf("Day %d | Cycle %d/%d | Collecting...\n",
                dayNumber, daily.cyclesCompleted, CYCLES_PER_DAY);
  Serial.println("════════════════════════════════════════════════════════════════");
}

// ============== Main Loop ==============
void loop() {
  unsigned long currentTime = millis();

  // Handle serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();

    switch (cmd) {
      case 'c': case 'C': clearRainfallSensor(); break;
      case 'b': case 'B': resetRainfallTracking(); break;
      case 's': case 'S': clearSunshineDuration(); break;
      case 'r': case 'R': resetTempTracking(); Serial.println("[TEMP] Reset"); break;
      case 't': case 'T': testAllSensors(); break;
      case 'd': case 'D': showDailyProgress(); break;
      case 'f': case 'F':
        Serial.println("[FORCE] Generating daily summary...");
        printDailySummary();
        break;
      case 'x': case 'X':
        Serial.println("[SPIFFS] Clearing all data and restarting...");
        clearSPIFFSDaily();
        dayNumber = 1;
        cycleNumber = 0;
        initializeDailyData();
        Serial.println("[SPIFFS] Fresh start - Day 1");
        break;
    }
  }

  // ===== Read Temperature Every 5 Seconds =====
  if (currentTime - lastTempRead >= TEMP_READ_INTERVAL) {
    lastTempRead = currentTime;

    bool atmosOk = readAtmos();

    if (atmosOk) {
      if (currentTemp < tempMin) tempMin = currentTemp;
      if (currentTemp > tempMax) tempMax = currentTemp;
      tempSum += currentTemp;
      tempReadCount++;

      Serial.printf("[5s] Temp: %.1f C (Min: %.1f Max: %.1f) | Cnt: %d | Day %d Cycle %d/%d\n",
                    currentTemp, tempMin, tempMax, tempReadCount,
                    dayNumber, daily.cyclesCompleted + 1, CYCLES_PER_DAY);
    } else {
      Serial.println("[5s] Temp: SENSOR ERROR");
    }

    bool solarOk = readSolar();
    if (solarOk) {
      updateSunshineDuration();
      Serial.printf("     Solar: %.0f W/m2 | Sun: %s | Total: %.2f min\n",
                    solarRadiation,
                    isSunshining ? "YES" : "NO",
                    getSunshineDurationMinutes());
    }
  }

  // ===== Send All Data Every 15 Seconds (1 Cycle) =====
  if (currentTime - lastDataSend >= SEND_INTERVAL) {
    lastDataSend = currentTime;
    cycleNumber++;

    Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
    Serial.printf("║  CYCLE %d/%d - DAY %d                                         \n",
                  daily.cyclesCompleted + 1, CYCLES_PER_DAY, dayNumber);
    Serial.println("╚══════════════════════════════════════════════════════════════╝");

    // Read all sensors
    bool windOk = readWind();
    delay(100);
    bool rainOk = readRain();
    delay(100);
    bool solarOk = readSolar();
    delay(100);
    bool atmosOk = readAtmos();

    if (solarOk) updateSunshineDuration();

    float cycleSunshine = getCycleSunshineMinutes();

    // Calculate VPD
    float vpd = 0;
    if (atmosOk && !isnan(currentTemp) && !isnan(currentHumidity)) {
      vpd = calculateVPD(currentTemp, currentHumidity);
    }

    float tempAvg = (tempReadCount > 0) ? (tempSum / tempReadCount) : currentTemp;
    bool hadErrors = (!windOk || !rainOk || !solarOk || !atmosOk);

    // Print cycle data
    Serial.println("\n┌──────────────────────────────────────────────────────────┐");
    Serial.println("│  SENSOR READINGS                                         │");
    Serial.println("├──────────────────────────────────────────────────────────┤");

    if (tempReadCount > 0) {
      Serial.printf("│  Temp Min:     %.1f C | Max: %.1f C | Avg: %.1f C (%d)\n",
                    tempMin, tempMax, tempAvg, tempReadCount);
    } else {
      Serial.println("│  Temperature:  NO DATA");
    }

    if (atmosOk) {
      Serial.printf("│  Humidity:     %.1f %%RH\n", currentHumidity);
      Serial.printf("│  Pressure:     %.1f hPa\n", currentPressure);
    } else {
      Serial.println("│  Atmos:        SENSOR ERROR");
    }

    if (windOk) {
      Serial.printf("│  Wind:         %.1f m/s @ %.0f deg\n", windSpeed, windDir);
    } else {
      Serial.println("│  Wind:         SENSOR ERROR");
    }

    if (solarOk) {
      Serial.printf("│  Solar:        %.0f W/m2 | Sun(cycle): %.3f min | Sun(total): %.2f min\n",
                    solarRadiation, cycleSunshine, getSunshineDurationMinutes());
    } else {
      Serial.println("│  Solar:        SENSOR ERROR");
    }

    if (rainOk) {
      Serial.printf("│  Rain(cycle):  %.1f mm | Rain(total): %.1f mm\n", rainfallCycle, rainfallTotal);
    } else {
      Serial.println("│  Rain:         SENSOR ERROR");
    }

    if (vpd > 0) {
      String vpdStatus;
      if (vpd < 0.4) vpdStatus = "Low (mold risk)";
      else if (vpd < 0.8) vpdStatus = "Propagation";
      else if (vpd < 1.2) vpdStatus = "Vegetative";
      else if (vpd < 1.6) vpdStatus = "Flowering";
      else vpdStatus = "HIGH (stress)";
      Serial.printf("│  VPD:          %.3f kPa - %s\n", vpd, vpdStatus.c_str());
    }

    Serial.println("└──────────────────────────────────────────────────────────┘");

    // CSV for logging
    Serial.printf("CSV,%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.0f,%.3f,%.1f,%.1f,%.3f,%.1f\n",
                  dayNumber, daily.cyclesCompleted + 1,
                  tempMin, tempMax, currentHumidity,
                  windSpeed, windDir, solarRadiation,
                  cycleSunshine, rainfallCycle, rainfallTotal,
                  vpd, currentPressure);

    // Update daily accumulators
    updateDailyAccumulators(tempMin, tempMax, tempAvg,
                            currentHumidity, currentPressure,
                            windSpeed, solarRadiation, cycleSunshine,
                            rainfallCycle, vpd, hadErrors);

    // Reset cycle tracking
    resetTempTracking();
    cycleStartSunshine = sunshineDurationMs;
    updateRainfallBaseline();

    // Save to SPIFFS
    saveDailyToSPIFFS();
    saveCycleBackup();

    // Progress bar
    int progress = (daily.cyclesCompleted * 50) / CYCLES_PER_DAY;
    Serial.print("[");
    for (int i = 0; i < 50; i++) {
      Serial.print(i < progress ? "#" : "-");
    }
    Serial.printf("] %d/%d (%.0f%%)\n\n",
                  daily.cyclesCompleted, CYCLES_PER_DAY,
                  (daily.cyclesCompleted * 100.0) / CYCLES_PER_DAY);

    // ===== Check if day is complete (96 cycles) =====
    if (daily.cyclesCompleted >= CYCLES_PER_DAY) {
      printDailySummary();

      // Start new day
      dayNumber++;
      initializeDailyData();
      clearSPIFFSDaily();

      Serial.println("════════════════════════════════════════════════════════════════");
      Serial.printf("Starting NEW DAY %d | Cycle 0/%d\n", dayNumber, CYCLES_PER_DAY);
      Serial.println("════════════════════════════════════════════════════════════════\n");
    }
  }

  delay(100);
}

// ============== Show Daily Progress ==============
void showDailyProgress() {
  float tempAvg = (daily.dayTempCount > 0) ? (daily.dayTempSum / daily.dayTempCount) : 0;
  float humAvg = (daily.dayHumidityCount > 0) ? (daily.dayHumiditySum / daily.dayHumidityCount) : 0;
  float windAvg = (daily.dayWindCount > 0) ? (daily.dayWindSpeedSum / daily.dayWindCount) : 0;
  float vpdAvg = (daily.dayVPDCount > 0) ? (daily.dayVPDSum / daily.dayVPDCount) : 0;

  Serial.println("\n=== DAILY PROGRESS (so far) ===");
  Serial.printf("Day %d | Cycle %d/%d (%.0f%%)\n",
                dayNumber, daily.cyclesCompleted, CYCLES_PER_DAY,
                (daily.cyclesCompleted * 100.0) / CYCLES_PER_DAY);
  Serial.printf("Temp: Min=%.1f Max=%.1f Avg=%.1f C\n",
                daily.dayTempMin, daily.dayTempMax, tempAvg);
  Serial.printf("Humidity: Avg=%.1f%% | Wind: Avg=%.1f Max=%.1f m/s\n",
                humAvg, windAvg, daily.dayWindSpeedMax);
  Serial.printf("Sunshine: %.2f min | Rain: %.1f mm | VPD: %.3f kPa\n",
                daily.daySunshineMinutes, daily.dayRainfallTotal, vpdAvg);
  Serial.printf("Errors: %d cycles\n", daily.cyclesWithErrors);
  Serial.println("================================\n");
}

// ============== Test All Sensors ==============
void testAllSensors() {
  Serial.println("\n╔══════════════════════════════════════════════════════════════╗");
  Serial.println("║  MANUAL SENSOR TEST                                          ║");
  Serial.println("╚══════════════════════════════════════════════════════════════╝\n");

  Serial.printf("Testing Atmospheric (0x%02X)... ", ATMOS_ADDR);
  if (readAtmos()) {
    Serial.printf("OK | T:%.1f C  H:%.1f%%  P:%.1f hPa\n", currentTemp, currentHumidity, currentPressure);
  } else {
    Serial.println("FAILED");
  }
  delay(200);

  Serial.printf("Testing Wind (0x%02X)... ", WIND_ADDR);
  if (readWind()) {
    Serial.printf("OK | Speed:%.1f m/s  Dir:%.0f deg\n", windSpeed, windDir);
  } else {
    Serial.println("FAILED");
  }
  delay(200);

  Serial.printf("Testing Solar (0x%02X)... ", SOLAR_ADDR);
  if (readSolar()) {
    Serial.printf("OK | Radiation:%.0f W/m2 | Sun:%s\n",
                  solarRadiation,
                  solarRadiation >= SUNSHINE_THRESHOLD ? "YES" : "NO");
  } else {
    Serial.println("FAILED");
  }
  delay(200);

  Serial.printf("Testing Rain (0x%02X)... ", RAIN_ADDR);
  if (readRain()) {
    Serial.printf("OK | Total:%.1f mm  Delta:%.1f mm\n", rainfallTotal, rainfallCycle);
  } else {
    Serial.println("FAILED");
  }

  if (currentTemp > 0 && currentHumidity > 0) {
    float vpd = calculateVPD(currentTemp, currentHumidity);
    Serial.printf("\nVPD: %.3f kPa\n", vpd);
  }

  Serial.printf("\nSPIFFS: %s | Day %d | Cycle %d/%d\n",
                spiffsReady ? "READY" : "NOT AVAILABLE",
                dayNumber, daily.cyclesCompleted, CYCLES_PER_DAY);
  Serial.println("\n════════════════════════════════════════════════════════════════\n");
}
