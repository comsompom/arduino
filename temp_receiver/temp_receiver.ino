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
  
  Serial.println("LoRa Temp Receiver");

  // --- Initialize I2C ---
  Wire.begin();
  
  // Initialize LCD
  lcd = LiquidCrystal_I2C(0x27, 16, 2);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Temp Receiver");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  Serial.println("LCD OK");

  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa failed!");
    lcd.clear();
    lcd.print("LoRa Failed!");
    while (1);
  }
  
  // Set LoRa parameters
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0x34);
  
  Serial.println("LoRa OK");
  
  // Show initial message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("No Data");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  Serial.println("Ready!");
}

void loop() {
  // Try to parse a packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    Serial.print("Received: ");

    String receivedString = "";
    while (LoRa.available()) {
      receivedString += (char)LoRa.read();
    }

    Serial.println(receivedString);
    rssi = LoRa.packetRssi();
    lastUpdateTime = millis();

    // --- Parse the received string ---
    // Expected format: "BMP:23.45,T1:24.12,T2:25.67" or "TEST:123"
    bool dataValid = false;
    
    // Check for test message first
    if (receivedString.indexOf("TEST:") != -1) {
      Serial.println("Test OK!");
      dataValid = true;
      
      // Show test message on LCD
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TEST OK");
      lcd.setCursor(0, 1);
      lcd.print(receivedString);
    }
    // Check for temperature data
    else if (receivedString.indexOf("BMP:") != -1 && 
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
      
      dataValid = true;
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
  
  // Show "no data" message initially
  if (lastUpdateTime == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("No Data");
    lcd.setCursor(0, 1);
    lcd.print("Waiting...");
  }
  
  delay(100);
}
