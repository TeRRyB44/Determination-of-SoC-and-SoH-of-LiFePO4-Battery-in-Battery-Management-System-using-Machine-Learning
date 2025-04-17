#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ADC Pins for voltage
const int cell1Pin = 26;
const int cell2Pin = 27;
const int cell3Pin = 14;
const int cell4Pin = 12;

// Thermistor Pin
#define THERMISTOR_PIN 13

// Current sensor Pin
#define SENSOR_PIN 39

// Relay Pin (normally closed)
#define RELAY_PIN 19

// Constants
#define ADC_MAX 4095.0
#define ADC_VREF 3.3
#define MV_PER_AMP 100.0
#define SERIES_RESISTOR 9130.0

// Voltage divider ratios and calibration factors
const double cellRatios[] = {
    (560000.0 + 100000.0) / 100000.0,    // ~7.012
    (560000.0 + 100000.0) / 95200.0,     // ~6.955
    (569000.0 + 220000.0) / 220000.0,    // ~3.586
    (560000.0 + 560000.0) / 555000.0     // ~2.018
};

const double calibrationFactors[] = {
    0.1259,     // 4-cell (cell1Pin)
    0.1093,     // 3-cell (cell2Pin)
    0.13,       // 2-cell (cell3Pin)
    2.85        // 1-cell (cell4Pin)
};

// Steinhart-Hart Coefficients
const float C1 = 1.009249522e-03;
const float C2 = 2.378405444e-04;
const float C3 = 2.019202697e-07;

// SoH Estimation
double observedMaxVoltage = 0.0;
const double nominalFullVoltage = 16.8;  // Adjust based on chemistry (4.2V x 4 cells)

void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);  // Relay initially ON (connected)
}

// Convert ADC reading to calibrated voltage
double getCalibratedVoltage(int adcValue, double ratio, double correctionFactor) {
  double voltage = (adcValue * ADC_VREF / ADC_MAX) * ratio;
  return voltage + (voltage * correctionFactor);
}

// Get temperature in Celsius
float readTemperature() {
  int rawADC = analogRead(THERMISTOR_PIN);
  float resistance = SERIES_RESISTOR * ((ADC_MAX / (float)rawADC) - 1.0);
  float logR = log(resistance);
  float temperatureKelvin = 1.0 / (C1 + C2 * logR + C3 * logR * logR * logR);
  return temperatureKelvin - 273.15;
}

// Get current in Amperes
float readCurrent() {
  int rawADC = analogRead(SENSOR_PIN);
  float voltage = (rawADC * ADC_VREF) / ADC_MAX;
  float zeroCurrentVoltage = (1808.0 * ADC_VREF) / ADC_MAX;  // Adjust zero offset
  float current = (voltage - zeroCurrentVoltage) / 0.100;    // 100mV/A
  return current;
}

// Polynomial regression model for SoC prediction
double calculateSoC(float voltage, float current, float temperature) {
  return (
    0.3848 * voltage +
    -0.3341 * current +
    -0.5391 * temperature +
    0.6682 * voltage * voltage +
    0.3305 * voltage * current +
    0.1143 * voltage * temperature +
    0.1519 * current * current +
    -0.0540 * current * temperature +
    0.5768 * temperature * temperature +
    0.0705
  );
}

// Voltage degradation-based SoH estimation
double calculateSoH(double totalVoltage) {
  if (totalVoltage > observedMaxVoltage) {
    observedMaxVoltage = totalVoltage;
  }
  double soh = (observedMaxVoltage / nominalFullVoltage) * 100.0;
  return constrain(soh, 0.0, 100.0);
}

void loop() {
  // Voltage Reading (updated block)
  int adcValues[] = {
    analogRead(cell1Pin),
    analogRead(cell2Pin),
    analogRead(cell3Pin),
    analogRead(cell4Pin)
  };

  double voltages[4];
  for (int i = 0; i < 4; i++) {
    voltages[i] = getCalibratedVoltage(adcValues[i], cellRatios[i], calibrationFactors[i]);
  }

  Serial.printf("Cell Voltage of Four cells: %.6f V\n", voltages[0]);
  Serial.printf("Cell Voltage of Three cells: %.6f V\n", voltages[1]);
  Serial.printf("Cell Voltage of Two cells: %.6f V\n", voltages[2]);
  Serial.printf("Cell Voltage of a Single cell: %.6f V\n", voltages[3]);

  double individualCells[4] = {
    voltages[0] - voltages[1],
    voltages[1] - voltages[2],
    voltages[2] - voltages[3],
    voltages[3]
  };

  Serial.printf("ADC Values: %d , %d , %d , %d \n", adcValues[0], adcValues[1], adcValues[2], adcValues[3]);
  Serial.printf("Individual Cell Voltages: %.6f V, %.6f V, %.6f V, %.6f V\n",
                individualCells[0], individualCells[1], individualCells[2], individualCells[3]);

  double totalVoltage = voltages[0];  // Total 4-cell voltage

  // Read temperature and current
  float temperatureC = readTemperature();
  float currentA = readCurrent();

  // Calculate SoC and SoH
  double soc = calculateSoC(totalVoltage, currentA, temperatureC);
  soc = constrain(soc * 100.0, 0.0, 100.0);  // Model outputs normalized SoC [0-1]

  double soh = calculateSoH(totalVoltage);

  // Print more info
  Serial.printf("Temperature: %.2f °C\n", temperatureC);
  Serial.printf("Current: %.2f A\n", currentA);
  Serial.printf("State of Charge (SoC): %.2f %%\n", soc);
  Serial.printf("State of Health (SoH): %.2f %%\n", soh);

  // Safety check
  bool safeVoltage = (totalVoltage >= 10.0 && totalVoltage <= 13.6);
  bool safeCurrent = (abs(currentA) <= 20.0);
  bool safeTemperature = (temperatureC >= -20.0 && temperatureC <= 55.0);

  if (safeVoltage && safeCurrent && safeTemperature) {
    digitalWrite(RELAY_PIN, HIGH);  // Relay closed (connected)
    Serial.println("BMS STATUS: SAFE ✅");
  } else {
    digitalWrite(RELAY_PIN, LOW); // Relay open (disconnected)
    Serial.println("BMS STATUS: UNSAFE ❌ → Relay OPENED");
  }

  delay(1000);
}