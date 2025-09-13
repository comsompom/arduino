#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// --- LoRa Pin Definitions ---
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

// --- LoRa Frequency ---
// 433E6 for Asia (433MHz), 868E6 for Europe, 915E6 for North America
#define LORA_FREQUENCY 433E6 

// --- TMP36 Sensor Pin Definitions ---
#define TMP36_PIN1 A0  // First TMP36 sensor
#define TMP36_PIN2 A1  // Second TMP36 sensor

// Create an object for the BMP085 sensor
Adafruit_BMP085 bmp;

// Variables to store sensor status
bool bmpSensorFound = false;

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
  
  Serial.println("LoRa Temp Transmitter");

  // Initialize I2C
  Wire.begin();
  
  // Initialize the BMP085 sensor
  if (!bmp.begin()) {
    Serial.println("BMP085 not found - using TMP36 only");
    bmpSensorFound = false;
  } else {
    bmpSensorFound = true;
    Serial.println("BMP085 OK");
  }

  // Wait for sensors to stabilize
  delay(1000);
  
  // Perform calibration based on BMP085 sensor (if available)
  if (bmpSensorFound) {
    float bmpReferenceTemp = bmp.readTemperature();
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
  
  // Set LoRa parameters
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x34);
  
  Serial.println("LoRa OK");
  
  // Send test message
  LoRa.beginPacket();
  LoRa.print("TEST:OK");
  LoRa.endPacket();
  
  Serial.println("Ready!");
}

void loop() {
  // Read temperatures from all sensors
  float bmpTemp = 0.0;
  float tmp36Temp1 = readTMP36(TMP36_PIN1, tmp36Calibration1);
  float tmp36Temp2 = readTMP36(TMP36_PIN2, tmp36Calibration2);
  
  // Read BMP085 temperature if sensor is available
  if (bmpSensorFound) {
    bmpTemp = bmp.readTemperature();
    if (isnan(bmpTemp)) {
      bmpTemp = 0.0;
    }
  }

  // Print basic info to Serial
  Serial.print("T1:");
  Serial.print(tmp36Temp1, 1);
  Serial.print(" T2:");
  Serial.print(tmp36Temp2, 1);
  if (bmpSensorFound) {
    Serial.print(" BMP:");
    Serial.print(bmpTemp, 1);
  }
  Serial.println();

  // Create a data string to send (format: "BMP:23.45,T1:24.12,T2:25.67")
  String dataString = "BMP:" + String(bmpTemp, 2) + 
                      ",T1:" + String(tmp36Temp1, 2) + 
                      ",T2:" + String(tmp36Temp2, 2);
  
  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  
  Serial.println("Sent: " + dataString);
  
  // Wait 10 seconds before sending the next packet
  delay(3000);
}
