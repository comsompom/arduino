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

// Create an object for the BMP085 sensor
Adafruit_BMP085 bmp;

// Variable to store sensor status
bool sensorFound = false;

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("=== LoRa BMP085 Temperature Transmitter ===");
  Serial.println("Initializing BMP085 sensor...");

  // Initialize I2C
  Wire.begin();
  
  // Initialize the BMP085 sensor
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
  
  sensorFound = true;
  Serial.println("BMP085 sensor initialized successfully!");

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
  Serial.println("Reading temperature every 10 seconds and transmitting via LoRa...");
  Serial.println();
}

void loop() {
  if (!sensorFound) {
    Serial.println("Sensor not found, cannot continue...");
    delay(5000);
    return;
  }

  // Read the temperature from the BMP085 sensor
  float temperature = bmp.readTemperature();
  
  // Check if temperature reading is valid
  if (isnan(temperature)) {
    Serial.println("ERROR: Invalid temperature reading!");
    delay(1000);
    return;
  }

  // Print to Serial monitor for debugging
  Serial.println("--- Temperature Reading ---");
  Serial.print("BMP085 Temperature: ");
  Serial.print(temperature, 2);
  Serial.println(" °C");

  // Create a data string to send (format: "T:23.45")
  String dataString = "T:" + String(temperature, 2);
  
  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print(dataString);
  LoRa.endPacket();
  
  Serial.println("Sent LoRa packet: " + dataString);
  Serial.println("Packet sent successfully!");
  Serial.println("Waiting 5 seconds before next transmission...");
  Serial.println();
  
  // Wait 5 seconds before sending the next packet
  delay(5000);
}
