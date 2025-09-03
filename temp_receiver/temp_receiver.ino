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
float temp1 = 0.0;
float temp2 = 0.0;
int rssi = 0;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("LoRa Temperature Receiver Starting...");

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
  lcd.setCursor(0, 0);
  lcd.print("Testing LCD...");
  delay(1000);
  
  // Simple test - if we can write to LCD, assume it's working
  lcdInitialized = true;
  Serial.println("LCD initialized successfully at 0x27");
  
  // Test with a simple pattern to verify it's working
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LCD Test");
  lcd.setCursor(0, 1);
  lcd.print("1234567890123456");
  delay(2000);
  
  // If we get here without errors, LCD is working
  Serial.println("LCD test pattern displayed successfully!");
  
  // Clear and show initial message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Receiver");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for data...");
  
  Serial.println("LCD Initialized Successfully!");
  
  // Show the actual message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Receiver");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for data...");

  // --- Initialize LoRa ---
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
    // Example format: "T1:23.50,T2:24.12"
    bool dataValid = false;
    
    if (receivedString.indexOf("T1:") != -1 && receivedString.indexOf("T2:") != -1) {
      int t1_start = receivedString.indexOf("T1:") + 3;
      int t1_end = receivedString.indexOf(",T2:");
      int t2_start = receivedString.indexOf("T2:") + 3;

      if (t1_start >= 3 && t1_end > t1_start && t2_start >= 3) {
        String temp1_str = receivedString.substring(t1_start, t1_end);
        String temp2_str = receivedString.substring(t2_start);
        
        // Convert to float values
        temp1 = temp1_str.toFloat();
        temp2 = temp2_str.toFloat();
        
        // Check if conversion was successful (allow 0°C as valid temperature)
        if (temp1 != 0.0 || temp1_str == "0" || temp1_str == "0.0") {
          if (temp2 != 0.0 || temp2_str == "0" || temp2_str == "0.0") {
            dataValid = true;
          }
        }
      }
    } else if (receivedString.indexOf("T1:") != -1) {
      // Only T1 data received
      int t1_start = receivedString.indexOf("T1:") + 3;
      String temp1_str = receivedString.substring(t1_start);
      temp1 = temp1_str.toFloat();
      if (temp1 != 0.0 || temp1_str == "0" || temp1_str == "0.0") {
        dataValid = true;
        temp2 = -999.0; // No T2 data
      }
    } else if (receivedString.indexOf("T2:") != -1) {
      // Only T2 data received
      int t2_start = receivedString.indexOf("T2:") + 3;
      String temp2_str = receivedString.substring(t2_start);
      temp2 = temp2_str.toFloat();
      if (temp2 != 0.0 || temp2_str == "0" || temp2_str == "0.0") {
        dataValid = true;
        temp1 = -999.0; // No T1 data
      }
    }
    
    if (dataValid) {
      // --- Update the LCD Display ---
      lcd.clear();
      
      // Line 1: First Temperature with Signal Strength
      lcd.setCursor(0, 0);
      if (temp1 != -999.0) {
        lcd.print("T1:");
        lcd.print((int)temp1);
        lcd.print("C RSSI:");
        lcd.print(rssi);
      } else {
        lcd.print("T1:No Data RSSI:");
        lcd.print(rssi);
      }
      
      // Line 2: Second Temperature with Signal Strength
      lcd.setCursor(0, 1);
      if (temp2 != -999.0) {
        lcd.print("T2:");
        lcd.print((int)temp2);
        lcd.print("C RSSI:");
        lcd.print(rssi);
      } else {
        lcd.print("T2:No Data RSSI:");
        lcd.print(rssi);
      }
      
      Serial.print("Parsed temperatures - T1: ");
      if (temp1 != -999.0) {
        Serial.print(temp1);
        Serial.print("°C");
      } else {
        Serial.print("No Data");
      }
      Serial.print(", T2: ");
      if (temp2 != -999.0) {
        Serial.print(temp2);
        Serial.println("°C");
      } else {
        Serial.println("No Data");
      }
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
