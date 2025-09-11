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

// Function to read TMP36 temperature
float readTMP36(int pin) {
  // Read the raw analog value from the sensor (0-1023)
  int sensorVal = analogRead(pin);
  
  // Convert the raw value to a voltage (0.0V - 5.0V)
  float voltage = (sensorVal / 1024.0) * 5.0;
  
  // Convert the voltage to temperature in Celsius
  // Formula: Temp C = (Voltage - 0.5) * 100
  float temperatureC = (voltage - 0.5) * 100;
  
  return temperatureC;
}

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("=== LoRa Multi-Temperature Transmitter ===");
  Serial.println("Initializing sensors...");

  // Initialize I2C
  Wire.begin();
  
  // Initialize the BMP085 sensor
  Serial.println("Initializing BMP085 sensor...");
  if (!bmp.begin()) {
    Serial.println("ERROR: Could not find a valid BMP085 sensor!");
    Serial.println("Check wiring:");
    Serial.println("1. VCC connected to Arduino 3.3V or 5V");
    Serial.println("2. GND connected to Arduino GND");
    Serial.println("3. SCL connected to Arduino A5");
    Serial.println("4. SDA connected to Arduino A4");
    Serial.println();
    Serial.println("Press RESET button to try again...");
    while (1) {} // Halt the program if sensor is not found
  }
  
  bmpSensorFound = true;
  Serial.println("BMP085 sensor initialized successfully!");

  // Initialize TMP36 sensors
  Serial.println("Initializing TMP36 sensors...");
  Serial.println("TMP36 Sensor 1 connected to A0");
  Serial.println("TMP36 Sensor 2 connected to A1");
  
  // Test TMP36 sensors
  float temp1 = readTMP36(TMP36_PIN1);
  float temp2 = readTMP36(TMP36_PIN2);
  
  Serial.print("TMP36 Sensor 1 test reading: ");
  Serial.print(temp1, 2);
  Serial.println(" °C");
  
  Serial.print("TMP36 Sensor 2 test reading: ");
  Serial.print(temp2, 2);
  Serial.println(" °C");
  
  Serial.println("TMP36 sensors initialized successfully!");

  // --- Initialize LoRa ---
  Serial.println("Initializing LoRa transmitter...");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    while (1); // Halt on failure
  }
  
  // Set LoRa parameters optimized for 433MHz operation
  LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  
  Serial.print("LoRa configured for ");
  Serial.print(LORA_FREQUENCY / 1000000);
  Serial.println(" MHz");
  
  Serial.println("LoRa Initialized Successfully!");
  Serial.println("Setup Complete!");
  Serial.println("Reading temperatures every 10 seconds and transmitting via LoRa...");
  Serial.println();
}

void loop() {
  if (!bmpSensorFound) {
    Serial.println("BMP085 sensor not found, cannot continue...");
    delay(5000);
    return;
  }

  // Read temperatures from all sensors
  float bmpTemp = bmp.readTemperature();
  float tmp36Temp1 = readTMP36(TMP36_PIN1);
  float tmp36Temp2 = readTMP36(TMP36_PIN2);
  
  // Check if BMP085 reading is valid
  if (isnan(bmpTemp)) {
    Serial.println("ERROR: Invalid BMP085 temperature reading!");
    delay(1000);
    return;
  }

  // Print to Serial monitor for debugging
  Serial.println("--- Temperature Readings ---");
  Serial.print("BMP085 Temperature: ");
  Serial.print(bmpTemp, 2);
  Serial.println(" °C");
  
  Serial.print("TMP36 Sensor 1: ");
  Serial.print(tmp36Temp1, 2);
  Serial.println(" °C");
  
  Serial.print("TMP36 Sensor 2: ");
  Serial.print(tmp36Temp2, 2);
  Serial.println(" °C");

  // Create a data string to send (format: "BMP:23.45,T1:24.12,T2:25.67")
  String dataString = "BMP:" + String(bmpTemp, 2) + 
                      ",T1:" + String(tmp36Temp1, 2) + 
                      ",T2:" + String(tmp36Temp2, 2);
  
  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  
  Serial.println("Sent LoRa packet: " + dataString);
  Serial.println("Packet sent successfully!");
  Serial.println("Waiting 10 seconds before next transmission...");
  Serial.println();
  
  // Wait 3 seconds before sending the next packet
  delay(3000);
}