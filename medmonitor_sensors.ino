/*
  MedMonitor integrated firmware
  ESP32 + AD8232 + MAX30102 + BMP280 + NTC

  Pins (classic ESP32 DevKit):
    AD8232 OUTPUT -> GPIO34
    AD8232 LO+    -> GPIO26
    AD8232 LO-    -> GPIO27
    MAX30102 SDA  -> GPIO21
    MAX30102 SCL  -> GPIO22
    BMP280 SDA    -> GPIO21 (shared I2C bus)
    BMP280 SCL    -> GPIO22 (shared I2C bus)
    NTC divider midpoint -> GPIO35

  NTC divider assumed:
    3.3V -> NTC -> GPIO35 -> 10k fixed resistor -> GND
  Set NTC_ON_TOP=false if your divider is wired the opposite way.

  Serial: 115200 baud
    ECG,<filtered_adc>       at 125 samples/sec
    ECG,LEADOFF              when electrode leads are disconnected
    DATA,<13 fields>         once per second

  Libraries:
    - Adafruit BMP280 Library
    - Adafruit Unified Sensor
    - Adafruit BusIO
    - SparkFun MAX3010x Pulse and Proximity Sensor Library
*/

#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <math.h>

// ---------------- PIN CONFIGURATION ----------------
static const int SDA_PIN = 21;
static const int SCL_PIN = 22;

static const int ECG_PIN = 34;
static const int ECG_LO_PLUS_PIN = 26;
static const int ECG_LO_MINUS_PIN = 27;

static const int NTC_PIN = 35;

// ---------------- NTC CONFIGURATION ----------------
#define NTC_ON_TOP true

static const float R_FIXED = 10000.0f;
static const float R_NTC_25 = 10000.0f;
static const float NTC_BETA = 3950.0f; // Replace with datasheet Beta if known.
static const float ADC_SUPPLY_MV = 3300.0f;

// ---------------- SENSOR OBJECTS ----------------
Adafruit_BMP280 bmp;
MAX30105 maxSensor;

bool bmpOK = false;
bool maxOK = false;

// ---------------- ECG CONFIGURATION ----------------
static const uint32_t ECG_PERIOD_US = 8000; // 125 Hz
static const int ECG_FILTER_SIZE = 3;

int ecgFilter[ECG_FILTER_SIZE] = {0};
int ecgFilterIndex = 0;
long ecgFilterSum = 0;
uint32_t nextEcgSampleUs = 0;
bool previousLeadOff = true;

// ---------------- MAX30102 CONFIGURATION ----------------
static const int32_t SPO2_BUFFER_SIZE = 100;

uint32_t irBuffer[SPO2_BUFFER_SIZE];
uint32_t redBuffer[SPO2_BUFFER_SIZE];

int spo2BufferCount = 0;
int samplesSinceCalculation = 0;

int32_t spo2Value = 0;
int32_t rawHeartRate = 0;

int8_t spo2Valid = 0;
int8_t heartRateValid = 0;

uint32_t latestIR = 0;
uint32_t latestRed = 0;

bool fingerPresent = false;

static const uint32_t FINGER_THRESHOLD = 50000;

// Heart-rate stabilizer:
// 1) reject isolated implausible jumps relative to the current estimate
// 2) median of the latest 5 accepted values
// 3) exponential smoothing of the median
static const int HR_HISTORY_SIZE = 5;
int hrHistory[HR_HISTORY_SIZE] = {0};
int hrHistoryCount = 0;
int hrHistoryIndex = 0;

float smoothedHeartRate = NAN;
static const float HR_EMA_ALPHA = 0.25f;
static const int HR_MIN = 40;
static const int HR_MAX = 200;
static const int HR_MAX_STEP = 25;

int displayedHeartRate = 0;

// ---------------- TIMING ----------------
unsigned long lastDataOutputMs = 0;

// ---------------- NTC READING ----------------
float readNTC(float &millivolts, float &resistance) {
  const int sampleCount = 32;
  uint32_t total = 0;

  for (int i = 0; i < sampleCount; i++) {
    total += analogReadMilliVolts(NTC_PIN);
    delayMicroseconds(150);
  }

  millivolts = total / (float)sampleCount;

  if (millivolts <= 20.0f ||
      millivolts >= ADC_SUPPLY_MV - 20.0f) {
    resistance = NAN;
    return NAN;
  }

  if (NTC_ON_TOP) {
    resistance = R_FIXED * (ADC_SUPPLY_MV - millivolts) / millivolts;
  } else {
    resistance = R_FIXED * millivolts / (ADC_SUPPLY_MV - millivolts);
  }

  if (!isfinite(resistance) || resistance <= 0.0f) {
    return NAN;
  }

  const float referenceKelvin = 298.15f;
  float inverseTemperature =
      (1.0f / referenceKelvin) +
      (log(resistance / R_NTC_25) / NTC_BETA);

  return (1.0f / inverseTemperature) - 273.15f;
}

// ---------------- HEART RATE FILTER ----------------
int medianOfHistory() {
  int sorted[HR_HISTORY_SIZE];
  for (int i = 0; i < hrHistoryCount; i++) {
    sorted[i] = hrHistory[i];
  }

  for (int i = 0; i < hrHistoryCount - 1; i++) {
    for (int j = i + 1; j < hrHistoryCount; j++) {
      if (sorted[j] < sorted[i]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  return sorted[hrHistoryCount / 2];
}

void resetHeartRateFilter() {
  hrHistoryCount = 0;
  hrHistoryIndex = 0;
  smoothedHeartRate = NAN;
  displayedHeartRate = 0;
}

void updateHeartRateFilter() {
  if (!fingerPresent || !heartRateValid) {
    return;
  }

  // Reject values outside the plausible range for this prototype.
  if (rawHeartRate < HR_MIN || rawHeartRate > HR_MAX) {
    return;
  }

  // Reject a single large jump once a stable estimate exists.
  if (isfinite(smoothedHeartRate) &&
      abs((int)rawHeartRate - (int)round(smoothedHeartRate)) > HR_MAX_STEP) {
    return;
  }

  hrHistory[hrHistoryIndex] = (int)rawHeartRate;
  hrHistoryIndex = (hrHistoryIndex + 1) % HR_HISTORY_SIZE;

  if (hrHistoryCount < HR_HISTORY_SIZE) {
    hrHistoryCount++;
  }

  int medianHR = medianOfHistory();

  if (!isfinite(smoothedHeartRate)) {
    smoothedHeartRate = medianHR;
  } else {
    smoothedHeartRate =
        HR_EMA_ALPHA * medianHR +
        (1.0f - HR_EMA_ALPHA) * smoothedHeartRate;
  }

  displayedHeartRate = (int)round(smoothedHeartRate);
}

// ---------------- MAX30102 SERVICE ----------------
void serviceMAX30102() {
  if (!maxOK) {
    return;
  }

  maxSensor.check();

  while (maxSensor.available()) {
    uint32_t red = maxSensor.getFIFORed();
    uint32_t ir = maxSensor.getFIFOIR();
    maxSensor.nextSample();

    latestRed = red;
    latestIR = ir;
    fingerPresent = (latestIR > FINGER_THRESHOLD);

    if (spo2BufferCount < SPO2_BUFFER_SIZE) {
      redBuffer[spo2BufferCount] = red;
      irBuffer[spo2BufferCount] = ir;
      spo2BufferCount++;

      if (spo2BufferCount == SPO2_BUFFER_SIZE) {
        maxim_heart_rate_and_oxygen_saturation(
          irBuffer, SPO2_BUFFER_SIZE, redBuffer,
          &spo2Value, &spo2Valid,
          &rawHeartRate, &heartRateValid
        );
        updateHeartRateFilter();
      }
    } else {
      memmove(irBuffer, irBuffer + 1,
              (SPO2_BUFFER_SIZE - 1) * sizeof(uint32_t));
      memmove(redBuffer, redBuffer + 1,
              (SPO2_BUFFER_SIZE - 1) * sizeof(uint32_t));

      irBuffer[SPO2_BUFFER_SIZE - 1] = ir;
      redBuffer[SPO2_BUFFER_SIZE - 1] = red;

      samplesSinceCalculation++;

      if (samplesSinceCalculation >= 25) {
        samplesSinceCalculation = 0;

        maxim_heart_rate_and_oxygen_saturation(
          irBuffer, SPO2_BUFFER_SIZE, redBuffer,
          &spo2Value, &spo2Valid,
          &rawHeartRate, &heartRateValid
        );

        updateHeartRateFilter();
      }
    }
  }

  if (!fingerPresent) {
    heartRateValid = 0;
    spo2Valid = 0;
    resetHeartRateFilter();
  }
}

// ---------------- ECG SERVICE ----------------
void serviceECG() {
  uint32_t nowUs = micros();

  if ((int32_t)(nowUs - nextEcgSampleUs) < 0) {
    return;
  }

  nextEcgSampleUs = nowUs + ECG_PERIOD_US;

  bool leadOff =
      (digitalRead(ECG_LO_PLUS_PIN) == HIGH) ||
      (digitalRead(ECG_LO_MINUS_PIN) == HIGH);

  if (leadOff) {
    if (!previousLeadOff) {
      Serial.println("ECG,LEADOFF");
    }
    previousLeadOff = true;
    return;
  }

  previousLeadOff = false;

  int rawECG = analogRead(ECG_PIN);

  ecgFilterSum -= ecgFilter[ecgFilterIndex];
  ecgFilter[ecgFilterIndex] = rawECG;
  ecgFilterSum += rawECG;
  ecgFilterIndex = (ecgFilterIndex + 1) % ECG_FILTER_SIZE;

  int filteredECG = ecgFilterSum / ECG_FILTER_SIZE;

  Serial.print("ECG,");
  Serial.println(filteredECG);
}

// ---------------- BMP280 INITIALIZATION ----------------
void initializeBMP280() {
  // Keep the same order and address used in the standalone sensor test.
  bmpOK = bmp.begin(0x76);

  if (!bmpOK) {
    delay(30);
    bmpOK = bmp.begin(0x77);
  }

  if (bmpOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500
    );
  }

  Serial.println(bmpOK ? "#BMP280:OK" : "#BMP280:NOT_FOUND");
}

// ---------------- MAX30102 INITIALIZATION ----------------
void initializeMAX30102() {
  maxOK = maxSensor.begin(Wire, I2C_SPEED_STANDARD);

  if (maxOK) {
    maxSensor.setup(60, 4, 2, 100, 411, 4096);
    maxSensor.setPulseAmplitudeRed(0x3F);
    maxSensor.setPulseAmplitudeIR(0x3F);
    maxSensor.setPulseAmplitudeGreen(0);
  }

  Serial.println(maxOK ? "#MAX30102:OK" : "#MAX30102:NOT_FOUND");
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(800);

  analogReadResolution(12);
  analogSetPinAttenuation(ECG_PIN, ADC_11db);
  analogSetPinAttenuation(NTC_PIN, ADC_11db);

  pinMode(ECG_LO_PLUS_PIN, INPUT);
  pinMode(ECG_LO_MINUS_PIN, INPUT);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  Serial.println("#MEDMONITOR_START");

  // BMP280 first, matching the standalone test that worked.
  initializeBMP280();

  // MAX30102 second, on the same I2C bus.
  initializeMAX30102();

  // Initialize ECG moving-average buffer with actual baseline samples.
  for (int i = 0; i < ECG_FILTER_SIZE; i++) {
    ecgFilter[i] = analogRead(ECG_PIN);
    ecgFilterSum += ecgFilter[i];
  }

  nextEcgSampleUs = micros();

  Serial.println(
    "#FORMAT:DATA,ntc_mV,ntc_ohms,ntc_C,bmp_C,pressure_hPa,IR,RED,finger,HR,HR_valid,SpO2,SpO2_valid"
  );
}

// ---------------- MAIN LOOP ----------------
void loop() {
  // Service the optical sensor frequently to avoid FIFO overflow.
  serviceMAX30102();

  // Sample ECG at 125 Hz and stream it separately from the one-second packet.
  serviceECG();

  if (millis() - lastDataOutputMs >= 1000) {
    lastDataOutputMs = millis();

    float ntcMv = NAN;
    float ntcOhms = NAN;
    float ntcC = readNTC(ntcMv, ntcOhms);

    float bmpC = NAN;
    float pressureHpa = NAN;

    if (bmpOK) {
      bmpC = bmp.readTemperature();
      pressureHpa = bmp.readPressure() / 100.0f;

      if (!isfinite(bmpC) || !isfinite(pressureHpa) ||
          bmpC < -40.0f || bmpC > 85.0f ||
          pressureHpa < 300.0f || pressureHpa > 1100.0f) {
        bmpC = NAN;
        pressureHpa = NAN;
      }
    }

    Serial.print("DATA,");
    Serial.print(isfinite(ntcMv) ? ntcMv : -999.0f, 1); Serial.print(",");
    Serial.print(isfinite(ntcOhms) ? ntcOhms : -999.0f, 1); Serial.print(",");
    Serial.print(isfinite(ntcC) ? ntcC : -999.0f, 2); Serial.print(",");
    Serial.print(isfinite(bmpC) ? bmpC : -999.0f, 2); Serial.print(",");
    Serial.print(isfinite(pressureHpa) ? pressureHpa : -999.0f, 2); Serial.print(",");
    Serial.print(latestIR); Serial.print(",");
    Serial.print(latestRed); Serial.print(",");
    Serial.print(fingerPresent ? 1 : 0); Serial.print(",");
    Serial.print(displayedHeartRate); Serial.print(",");
    Serial.print((fingerPresent && hrHistoryCount > 0) ? 1 : 0); Serial.print(",");
    Serial.print(spo2Value); Serial.print(",");
    Serial.println((fingerPresent && spo2Valid) ? 1 : 0);
  }
}
