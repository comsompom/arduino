#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// --- LoRa Pin Definitions ---
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

// --- LoRa Frequency ---
// 433E6 for Asia (433MHz), 868E6 for Europe, 915E6 for North America
#define LORA_FREQUENCY 433000000 

// --- LoRa Radio Profile (try different profiles if not receiving) ---
// 1: SF7/BW125/CR5 Sync 0x34 (default)
// 2: SF12/BW125/CR5 Sync 0x34 (longer range)
// 3: SF12/BW62.5/CR8 Sync 0x12 (compatibility)
#ifndef LORA_PROFILE
#define LORA_PROFILE 2
#endif

static void applyRadioProfile() {
  if (LORA_PROFILE == 1) {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x34);
  } else if (LORA_PROFILE == 2) {
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setSyncWord(0x34);
  } else {
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(8);
    LoRa.setSyncWord(0x34);
  }
}

static void printRadioProfile() {
  Serial.print("LoRa profile "); Serial.println(LORA_PROFILE);
}

// --- TMP36 Sensor Pin Definitions ---
#define TMP36_PIN1 A0  // First TMP36 sensor
#define TMP36_PIN2 A1  // Second TMP36 sensor

// Create an object for the BMP280 sensor
Adafruit_BMP280 bmp280;
static unsigned long sendSeq = 0;

// Variables to store sensor status
bool bmp280SensorFound = false;
uint8_t bmp280AddressUsed = 0x00;

// --- Low-level I2C helpers to probe BME/BMP chip ID and reset ---
static uint8_t i2cReadRegister(uint8_t i2cAddress, uint8_t reg) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }
  if (Wire.requestFrom(i2cAddress, (uint8_t)1) != 1) {
    return 0xFF;
  }
  return Wire.read();
}

static bool i2cWriteRegister(uint8_t i2cAddress, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2cAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static void softResetBME(uint8_t i2cAddress) {
  // BMP280 reset register 0xE0, reset command 0xB6
  i2cWriteRegister(i2cAddress, 0xE0, 0xB6);
  delay(5);
}

// Calibration offsets for TMP36 sensors
float tmp36Calibration1 = 0.0;  // Offset for TMP36 sensor 1
float tmp36Calibration2 = 0.0;  // Offset for TMP36 sensor 2

// Function to read TMP36 temperature (raw, without calibration)
float readTMP36Raw(int pin) {
  // Read the raw analog value from the sensor (0-1023)
  int sensorVal = analogRead(pin);
  
  // Convert the raw value to a voltage (0.0V - 5.0V)
  float voltage = (sensorVal / 1024.0) * 5.0;
  
  // Convert the voltage to temperature in Celsius
  // Formula: Temp C = (Voltage - 0.5) * 100
  float temperatureC = (voltage - 0.5) * 100;
  
  return temperatureC;
}

// Function to read TMP36 temperature (with calibration applied)
float readTMP36(int pin, float calibrationOffset) {
  float rawTemp = readTMP36Raw(pin);
  return rawTemp + calibrationOffset;
}

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("LoRa Temp Transmitter (BMP280)");

  // Initialize I2C
  Wire.begin();
  // Faster I2C can sometimes help with unstable modules; safe for most Arduino boards
  Wire.setClock(400000);
  delay(10);
  
  // Quick I2C scan to help diagnose address and wiring issues
  Serial.println("Scanning I2C bus...");
  uint8_t foundCount = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print(" - Found device at 0x");
      if (addr < 16) Serial.print('0');
      Serial.println(addr, HEX);
      foundCount++;
    }
  }
  if (foundCount == 0) {
    Serial.println("No I2C devices found. Check SDA/SCL wiring and power.");
  }
  
  // Initialize the BMP280 sensor (probe both 0x76 and 0x77, read chip ID, soft-reset, then begin)
  const uint8_t candidateAddrs[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2 && !bmp280SensorFound; i++) {
    uint8_t addr = candidateAddrs[i];
    // If device ACKs, read chip ID
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      uint8_t chipId = i2cReadRegister(addr, 0xD0);
      Serial.print("Probing 0x");
      Serial.print(addr, HEX);
      Serial.print(" - Chip ID: 0x");
      Serial.println(chipId, HEX);
      // Perform a soft reset before starting
      softResetBME(addr);
      // Initialize BMP280 driver
      if (bmp280.begin(addr)) {
        bmp280SensorFound = true;
        bmp280AddressUsed = addr;
      }
    }
  }
  if (!bmp280SensorFound) {
    // Final fallback: let library auto-detect
    if (bmp280.begin()) {
      bmp280SensorFound = true;
      bmp280AddressUsed = 0xFF;
    }
  }

  if (bmp280SensorFound) {
    Serial.print("BMP280 initialized at: ");
    if (bmp280AddressUsed == 0xFF) Serial.println("(library default)");
    else {
      Serial.print("0x");
      Serial.println(bmp280AddressUsed, HEX);
    }
  } else {
    Serial.println("BMP280 not found - using TMP36 only");
  }

  // Wait for sensors to stabilize
  delay(1000);
  
  // Perform calibration based on BMP280 sensor (if available)
  if (bmp280SensorFound) {
    float bmpReferenceTemp = bmp280.readTemperature();
    if (!isnan(bmpReferenceTemp)) {
      float tmp36Raw1 = readTMP36Raw(TMP36_PIN1);
      float tmp36Raw2 = readTMP36Raw(TMP36_PIN2);
      tmp36Calibration1 = bmpReferenceTemp - tmp36Raw1;
      tmp36Calibration2 = bmpReferenceTemp - tmp36Raw2;
      Serial.println("TMP36 calibrated");
    }
  }

  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa failed!");
    while (1);
  }
  
  // Use exact same settings as working test
  LoRa.setTxPower(14, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.disableCrc();
  Serial.print("Freq:433.00MHz  SF:7  BW:125kHz  CR:5");
  
  Serial.println("LoRa OK");
  
  // Send test message
  LoRa.beginPacket();
  LoRa.print("TEST:OK");
  LoRa.endPacket();
  
  Serial.println("Ready!");
}

void loop() {
  Serial.println("Loop start");
  
  // Read temperatures from all sensors
  float bmeTemp = 0.0;
  float tmp36Temp1 = readTMP36(TMP36_PIN1, tmp36Calibration1);
  float tmp36Temp2 = readTMP36(TMP36_PIN2, tmp36Calibration2);
  
  // Read BMP280 temperature if sensor is available
  if (bmp280SensorFound) {
    bmeTemp = bmp280.readTemperature();
    if (isnan(bmeTemp)) {
      bmeTemp = 0.0;
    }
  }

  // Print basic info to Serial
  Serial.print("T1:");
  Serial.print(tmp36Temp1, 1);
  Serial.print(" T2:");
  Serial.print(tmp36Temp2, 1);
  if (bmp280SensorFound) {
    Serial.print(" BMP:");
    Serial.print(bmeTemp, 1);
  }
  Serial.println();

  // Create a data string to send (format: "SEQ:n,BMP:23.45,T1:24.12,T2:25.67")
  String dataString;
  if (bmp280SensorFound) {
    dataString = String("SEQ:") + String(sendSeq++) + ",BMP:" + String(bmeTemp, 2) + ",T1:" + String(tmp36Temp1, 2) + ",T2:" + String(tmp36Temp2, 2);
  } else {
    dataString = String("SEQ:") + String(sendSeq++) + ",BMP:" + String(0.0, 2) + ",T1:" + String(tmp36Temp1, 2) + ",T2:" + String(tmp36Temp2, 2);
  }
  
  // Send LoRa packet
  Serial.println("About to send packet");
  LoRa.idle();
  delay(10); // Small delay to ensure idle mode is set
  
  if (LoRa.beginPacket()) {
    // Temporarily send test message to see if receiver can get it
    LoRa.print("TEST:LOOP");
    LoRa.endPacket();
    delay(10); // Small delay after transmission
    LoRa.idle(); // Return to idle mode after transmission
  } else {
    Serial.println("Failed to begin packet");
  }
  
  Serial.println("Sent: " + dataString);
  Serial.println("Loop end");
  
  // Wait before sending the next packet
  delay(1000);
}
