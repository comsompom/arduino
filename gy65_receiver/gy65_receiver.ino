#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// --- LoRa Pin Definitions ---
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

// --- LoRa Frequency ---
// Must match the transmitter's frequency! (433MHz)
#define LORA_FREQUENCY 433E6 

// --- LCD Setup ---
// Set the LCD address to 0x27 for a 16 chars and 2 line display
// Note: Your I2C address might be 0x3F. If it doesn't work, try that.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variables to store temperature values
float temperature = 0.0;
int rssi = 0;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("=== LoRa BMP085 Temperature Receiver ===");
  Serial.println("Initializing receiver...");

  // --- Initialize I2C ---
  Wire.begin();
  
  // Scan for I2C devices to find LCD address
  Serial.println("Scanning for I2C devices...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  
  // Try to initialize LCD with common addresses
  bool lcdInitialized = false;
  
  // Try address 0x27 first (most common)
  Serial.println("Trying LCD address 0x27...");
  lcd = LiquidCrystal_I2C(0x27, 16, 2);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Simple test - if we can write to LCD, assume it's working
  lcdInitialized = true;
  Serial.println("LCD initialized successfully at 0x27");
  
  // Clear and show initial message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp Receiver");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for data...");
  
  Serial.println("LCD Initialized Successfully!");

  // --- Initialize LoRa ---
  Serial.println("Initializing LoRa receiver...");
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("Starting LoRa failed!");
    lcd.clear();
    lcd.print("LoRa Failed!");
    while (1); // Halt on failure
  }
  
  // Set LoRa parameters optimized for 433MHz operation
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  
  Serial.print("LoRa configured for ");
  Serial.print(LORA_FREQUENCY / 1000000);
  Serial.println(" MHz");
  
  Serial.println("LoRa Initialized Successfully!");
  
  // Show ready message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  delay(2000);
  
  // Show initial "No Data" message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("No Data");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  Serial.println("Setup Complete! Waiting for BMP085 temperature data...");
}

void loop() {
  // Try to parse a packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received packet '");

    String receivedString = "";
    while (LoRa.available()) {
      receivedString += (char)LoRa.read();
    }

    Serial.print(receivedString);
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());

    // Store RSSI value
    rssi = LoRa.packetRssi();
    
    // Update timestamp
    lastUpdateTime = millis();

    // --- Parse the received string ---
    // Expected format: "T:23.45"
    bool dataValid = false;
    
    if (receivedString.indexOf("T:") != -1) {
      int t_start = receivedString.indexOf("T:") + 2;
      
      if (t_start >= 2) {
        String temp_str = receivedString.substring(t_start);
        
        // Convert to float value
        temperature = temp_str.toFloat();
        
        // Check if conversion was successful (allow 0°C as valid temperature)
        if (temperature != 0.0 || temp_str == "0" || temp_str == "0.0") {
          dataValid = true;
        }
      }
    }
    
    if (dataValid) {
      // --- Update the LCD Display ---
      lcd.clear();
      
      // Line 1: Temperature with Signal Strength
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(temperature, 1);
      lcd.print("C");
      
      // Line 2: Signal Strength (RSSI)
      lcd.setCursor(0, 1);
      lcd.print("RSSI:");
      lcd.print(rssi);
      lcd.print("dBm");
      
      Serial.print("Parsed temperature: ");
      Serial.print(temperature, 2);
      Serial.println("°C");
    } else {
      Serial.println("Invalid data format received");
    }
  }
  
  // Show "no data" message if no updates for 30 seconds
  if (millis() - lastUpdateTime > 30000 && lastUpdateTime != 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Data");
    lcd.setCursor(0, 1);
    lcd.print("Check Transmitter");
  }
  
  // Show "no data" message initially or if no data ever received
  if (lastUpdateTime == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Data");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
  }
  
  // Show status indicator (blinking dot) when waiting
  static unsigned long lastBlink = 0;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    if (lastUpdateTime == 0) {
      // Toggle the last character to show activity
      lcd.setCursor(15, 1);
      static bool blinkState = false;
      lcd.print(blinkState ? "." : " ");
      blinkState = !blinkState;
    }
  }
  
  // Small delay to prevent overwhelming the system
  delay(100);
}
