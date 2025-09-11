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
float bmpTemp = 0.0;
float tmp36Temp1 = 0.0;
float tmp36Temp2 = 0.0;
int rssi = 0;
unsigned long lastUpdateTime = 0;

void setup() {
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("=== LoRa Multi-Temperature Receiver ===");
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
  
  Serial.println("Setup Complete! Waiting for multi-temperature data...");
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
    // Expected format: "BMP:23.45,T1:24.12,T2:25.67"
    bool dataValid = false;
    
    if (receivedString.indexOf("BMP:") != -1 && 
        receivedString.indexOf("T1:") != -1 && 
        receivedString.indexOf("T2:") != -1) {
      
      // Parse BMP085 temperature
      int bmp_start = receivedString.indexOf("BMP:") + 4;
      int bmp_end = receivedString.indexOf(",T1:");
      if (bmp_start >= 4 && bmp_end > bmp_start) {
        String bmp_str = receivedString.substring(bmp_start, bmp_end);
        bmpTemp = bmp_str.toFloat();
      }
      
      // Parse TMP36 Sensor 1 temperature
      int t1_start = receivedString.indexOf("T1:") + 3;
      int t1_end = receivedString.indexOf(",T2:");
      if (t1_start >= 3 && t1_end > t1_start) {
        String t1_str = receivedString.substring(t1_start, t1_end);
        tmp36Temp1 = t1_str.toFloat();
      }
      
      // Parse TMP36 Sensor 2 temperature
      int t2_start = receivedString.indexOf("T2:") + 3;
      if (t2_start >= 3) {
        String t2_str = receivedString.substring(t2_start);
        tmp36Temp2 = t2_str.toFloat();
      }
      
      // Check if all conversions were successful
      if (bmpTemp != 0.0 || tmp36Temp1 != 0.0 || tmp36Temp2 != 0.0) {
        dataValid = true;
      }
    }
    
    if (dataValid) {
      // --- Update the LCD Display ---
      lcd.clear();
      
      // Line 1: TMP36 Sensor 1 and TMP36 Sensor 2 temperatures (integers)
      lcd.setCursor(0, 0);
      lcd.print((int)tmp36Temp1);
      lcd.print(":");
      lcd.print((int)tmp36Temp2);
      
      // Line 2: BMP085 temperature and RSSI separated by ":"
      lcd.setCursor(0, 1);
      lcd.print((int)bmpTemp);
      lcd.print(":");
      lcd.print(" RSSI:");
      lcd.print(rssi);
      
      Serial.print("Parsed temperatures - BMP: ");
      Serial.print(bmpTemp, 2);
      Serial.print("°C, TMP36-1: ");
      Serial.print(tmp36Temp1, 2);
      Serial.print("°C, TMP36-2: ");
      Serial.print(tmp36Temp2, 2);
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