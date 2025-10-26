//======================================================================
//  ESP32 GPS Tracker with I2C LCD and Buzzer
//  - Displays Latitude, Longitude, and Satellite count.
//  - Plays Pirates of the Caribbean melody when GPS fix is acquired.
//======================================================================

// 1. LIBRARIES
//----------------------------------------------------------------------
#include <Wire.h>                 // For I2C communication
#include <LiquidCrystal_I2C.h>    // For the I2C LCD (ESP32 compatible version)
#include <TinyGPS++.h>            // For parsing GPS data
#include <HardwareSerial.h>       // For using secondary serial ports on ESP32

// NOTE: If you get a warning about LiquidCrystal_I2C being AVR-only, 
// install the ESP32-compatible version by Frank de Brabander from Library Manager

// 2. CONFIGURATION & PIN DEFINITIONS
//----------------------------------------------------------------------
// LCD Configuration
// Note: Your I2C address might be 0x3F. If 0x27 doesn't work, try that.
LiquidCrystal_I2C lcd(0x27, 16, 2);

// GPS Configuration
// We will use Serial2 for the GPS module.
// ESP32's RX2 is GPIO 16, TX2 is GPIO 17
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// Buzzer Configuration
#define BUZZER_PIN 25

// 3. MELODY DEFINITIONS
//----------------------------------------------------------------------
// Notes frequencies (C4 to B5) - Extended range for pirate melody
#define NOTE_C4  262
#define NOTE_D4  294
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_D5  587
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_G5  784
#define NOTE_A5  880
#define NOTE_B5  988

// Pirates of the Caribbean - "He's a Pirate" melody (simplified version)
int melody[] = {
  NOTE_E4, NOTE_E4, NOTE_E4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_E4, NOTE_G4,
  NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_A4, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_A4, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_D5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_D5, NOTE_F5,
  NOTE_E5, NOTE_E5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_E5, NOTE_G5
};

int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4,  // First phrase
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4,  // Second phrase  
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4,  // Third phrase
  4, 4, 4, 4, 4, 4, 4, 4, 4, 4   // Fourth phrase
};

// 4. GLOBAL VARIABLES
//----------------------------------------------------------------------
bool melodyPlayed = false; // Flag to ensure melody only plays once
bool gpsModuleFound = false; // Flag to track if GPS module was detected during setup

//======================================================================
//  SETUP FUNCTION - Runs once at the beginning
//======================================================================
void setup() {
  // Start the main serial port for debugging (optional)
  Serial.begin(115200);
  Serial.println("ESP32 GPS Tracker Starting...");

  // Start the hardware serial port for the GPS module
  // The NEO-6M default baud rate is 9600
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("GPS Serial initialized on pins 16(RX), 17(TX)");

  // Setup the buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);
  Serial.println("Buzzer pin configured");

  // Initialize the LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  Serial.println("LCD initialized");

  // Display startup message
  lcd.setCursor(0, 0);
  lcd.print("GPS Tracker");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(1000);
  
  // Check if GPS module is responding
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS Module:");
  lcd.setCursor(0, 1);
  lcd.print("Checking...");
  delay(500);
  
  // Test GPS communication
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) { // Check for 3 seconds
    if (gpsSerial.available() > 0) {
      gpsModuleFound = true;
      break;
    }
    delay(100);
  }
  
  // Display GPS module status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS Module:");
  lcd.setCursor(0, 1);
  if (gpsModuleFound) {
    lcd.print("Found! Searching...");
    Serial.println("GPS module detected - starting search for satellites");
  } else {
    lcd.print("Not found!");
    Serial.println("Warning: GPS module not responding");
  }
  delay(2000);
  
  // Clear and show final status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS Tracker");
  lcd.setCursor(0, 1);
  if (gpsModuleFound) {
    lcd.print("Searching...");
  } else {
    lcd.print("No GPS signal");
  }
  
  Serial.println("Setup complete - waiting for GPS signal...");
}

//======================================================================
//  LOOP FUNCTION - Runs repeatedly
//======================================================================
void loop() {
  // This is the magic. It reads from the GPS serial port and feeds the
  // data to the TinyGPS++ object.
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      displayGpsInfo();
    }
  }

  // If no GPS data is received for 5 seconds, let the user know.
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 2000) { // Update every 2 seconds
      lcd.clear();
      lcd.setCursor(0, 0);
      if (gpsModuleFound) {
        lcd.print("Searching...");
        lcd.setCursor(0, 1);
        lcd.print("Waiting for fix");
      } else {
        lcd.print("GPS Module");
        lcd.setCursor(0, 1);
        lcd.print("Not detected!");
      }
      lastUpdate = millis();
      if (gpsModuleFound) {
        Serial.println("GPS module found but no fix yet - keep waiting");
      } else {
        Serial.println("GPS module not detected - check connections");
      }
    }
  }
}

//======================================================================
//  HELPER FUNCTIONS
//======================================================================

/**
 * @brief Displays the current GPS information on the LCD.
 */
void displayGpsInfo() {
  lcd.clear();
  lcd.setCursor(0, 0);

  // Check if we have a valid location
  if (gps.location.isValid()) {
    // Format coordinates to 4 decimal places and join with a semicolon
    String lat_str = String(gps.location.lat(), 4);
    String lng_str = String(gps.location.lng(), 4);
    lcd.print(lat_str + ";" + lng_str);
    
    // Debug output to Serial
    Serial.print("GPS Fix: Lat=");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", Lng=");
    Serial.print(gps.location.lng(), 6);
    Serial.print(", Sats=");
    Serial.println(gps.satellites.value());

    // Play melody if we have enough satellites and it hasn't been played yet
    if (gps.satellites.value() >= 4 && !melodyPlayed) {
      Serial.println("Playing Pirates of the Caribbean melody!");
      playMelody();
      melodyPlayed = true;
    }

  } else {
    lcd.print("Searching...");
  }

  lcd.setCursor(0, 1);
  lcd.print("Sats: ");
  if (gps.satellites.isValid()) {
    lcd.print(gps.satellites.value());
  } else {
    lcd.print("N/A");
  }
}


/**
 * @brief Plays the Pirates of the Caribbean melody on the buzzer.
 */
void playMelody() {
  int melodyLength = sizeof(melody) / sizeof(melody[0]);
  
  for (int thisNote = 0; thisNote < melodyLength; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    
    // Generate tone using ESP32's built-in tone function
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    delay(noteDuration + noteDuration * 0.1); // Shorter pause for faster melody
  }
  noTone(BUZZER_PIN); // Ensure buzzer is off
}