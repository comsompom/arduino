#include <SPI.h>
#include <LoRa.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- LoRa Pin Definitions ---
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

// --- LoRa Frequency ---
// 433E6 for Asia (433MHz), 868E6 for Europe, 915E6 for North America
#define LORA_FREQUENCY 433E6 

// --- Sensor Pin Definition ---
#define ONE_WIRE_BUS 4

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// DeviceAddress variables to hold the unique addresses of our sensors
DeviceAddress sensor1, sensor2;

// Variables to store sensor addresses
bool sensor1Found = false;
bool sensor2Found = false;

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("LoRa Temperature Transmitter Starting...");

  // --- Initialize LoRa ---
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

  // --- Initialize DS18B20 Sensors ---
  Serial.println("Initializing DS18B20 sensors on Pin 4...");
  sensors.begin();
  delay(1000); // Give sensors time to initialize
  
  // Try to set resolution for better accuracy
  sensors.setResolution(12); // 12-bit resolution (0.0625°C)
  
  // Locate devices on the bus
  int deviceCount = sensors.getDeviceCount();
  Serial.print("Found ");
  Serial.print(deviceCount);
  Serial.println(" temperature sensor(s)");
  
  // More robust sensor detection
  if (deviceCount >= 1) {
    sensor1Found = true;
    Serial.println("Sensor 1 will be read from index 0");
  } else {
    Serial.println("No sensors detected on the bus");
  }
  
  if (deviceCount >= 2) {
    sensor2Found = true;
    Serial.println("Sensor 2 will be read from index 1");
  } else if (deviceCount == 1) {
    Serial.println("Only 1 sensor found - will use single sensor mode");
  }
  
  if (deviceCount == 0) {
    Serial.println("No sensors found! Check wiring and 4.7kΩ pull-up resistor.");
    Serial.println("Make sure:");
    Serial.println("1. VCC (Red) wires from both sensors connected to Arduino 5V");
    Serial.println("2. GND (Black) wires from both sensors connected to Arduino GND");
    Serial.println("3. Data (Yellow) wires from both sensors connected to Arduino Pin 4");
    Serial.println("4. 4.7kΩ resistor between 5V and Pin 4");
    while (1); // Halt if no sensors found
  }
  
  Serial.println("Setup Complete!");
  
  // Debug: Print sensor addresses if found
  if (deviceCount > 0) {
    Serial.println("Sensor addresses found:");
    for (int i = 0; i < deviceCount; i++) {
      DeviceAddress tempAddress;
      if (sensors.getAddress(tempAddress, i)) {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        for (uint8_t j = 0; j < 8; j++) {
          if (tempAddress[j] < 16) Serial.print("0");
          Serial.print(tempAddress[j], HEX);
        }
        Serial.println();
      }
    }
  }
}

void loop() {
  // Request temperatures from all devices on the bus
  sensors.requestTemperatures(); 
  
  // Variables to store temperatures
  float temp1 = -999.0;
  float temp2 = -999.0;
  
  // Get temperatures by their index on the bus
  if (sensor1Found) {
    temp1 = sensors.getTempCByIndex(0);
    // Check if reading is valid
    if (temp1 == DEVICE_DISCONNECTED_C) {
      temp1 = -999.0;
      Serial.println("Error: Sensor 1 disconnected");
    }
  }
  
  if (sensor2Found) {
    temp2 = sensors.getTempCByIndex(1);
    // Check if reading is valid
    if (temp2 == DEVICE_DISCONNECTED_C) {
      temp2 = -999.0;
      Serial.println("Error: Sensor 2 disconnected");
    }
  }

  // Print to Serial monitor for debugging
  Serial.println("--- Temperature Readings ---");
  if (sensor1Found) {
    Serial.print("Sensor 1: ");
    if (temp1 != -999.0) {
      Serial.print(temp1, 2);
      Serial.println(" °C");
    } else {
      Serial.println("Error reading");
    }
  }
  
  if (sensor2Found) {
    Serial.print("Sensor 2: ");
    if (temp2 != -999.0) {
      Serial.print(temp2, 2);
      Serial.println(" °C");
    } else {
      Serial.println("Error reading");
    }
  }

  // Create a data string to send
  String dataString = "";
  if (sensor1Found && temp1 != -999.0) {
    dataString += "T1:" + String(temp1, 2);
  }
  if (sensor2Found && temp2 != -999.0) {
    if (dataString.length() > 0) dataString += ",";
    dataString += "T2:" + String(temp2, 2);
  }
  
  // Only send if we have valid data
  if (dataString.length() > 0) {
    // Send LoRa packet
    LoRa.beginPacket();
    LoRa.print(dataString);
    LoRa.endPacket();
    
    Serial.println("Sent LoRa packet: " + dataString);
    Serial.println("Packet sent successfully!");
  } else {
    Serial.println("No valid temperature data to send");
  }

  Serial.println("Waiting 10 seconds before next transmission...");
  Serial.println();
  
  // Wait 10 seconds before sending the next packet
  delay(10000);
}
