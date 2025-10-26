//======================================================================
//  ESP32 GPS & Gyro Tracker with I2C LCD and Buzzer
//  - Displays GPS Coordinates (Lat;Lon) and Gyroscope data (X;Y;Z).
//  - Plays melody when GPS fix is acquired.
//  - UPDATED to include MPU-6050 Gyroscope.
//======================================================================

// 1. LIBRARIES
//----------------------------------------------------------------------
#include <Wire.h>                 // For I2C communication
#include <LiquidCrystal_I2C.h>    // For the I2C LCD
#include <TinyGPS++.h>            // For parsing GPS data
#include <HardwareSerial.h>       // For using secondary serial ports on ESP32
#include <Adafruit_MPU6050.h>     // For the MPU-6050 Gyroscope/Accelerometer
#include <Adafruit_Sensor.h>      // Dependency for MPU-6050 library

// 2. CONFIGURATION & PIN DEFINITIONS
//----------------------------------------------------------------------
// LCD Configuration (Address 0x27 is common, but 0x3F is also possible)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// MPU-6050 Configuration
Adafruit_MPU6050 mpu;

// GPS Configuration (Using Serial2 on ESP32)
HardwareSerial gpsSerial(2); // RX=16, TX=17
TinyGPSPlus gps;

// Buzzer Configuration
#define BUZZER_PIN 25

// 3. MELODY DEFINITIONS (Same as before)
//----------------------------------------------------------------------
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

int melody[] = { NOTE_E4, NOTE_E4, NOTE_E4, NOTE_C4, NOTE_E4, NOTE_G4, NOTE_G4, NOTE_G4, NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_A4, NOTE_F4, NOTE_A4, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, NOTE_D5, NOTE_B4, NOTE_D5, NOTE_F5, NOTE_F5, NOTE_F5, NOTE_D5, NOTE_F5, NOTE_E5, NOTE_E5, NOTE_E5, NOTE_C5, NOTE_E5, NOTE_G5, NOTE_G5, NOTE_G5, NOTE_E5, NOTE_G5 };
int noteDurations[] = { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 };

// 4. GLOBAL VARIABLES
//----------------------------------------------------------------------
bool melodyPlayed = false;
bool gpsModuleFound = false;

//======================================================================
//  SETUP FUNCTION - Runs once at the beginning
//======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 GPS & Gyro Tracker Starting...");

  // Start I2C and initialize LCD
  Wire.begin(21, 22); // Explicitly start I2C on SDA=21, SCL=22
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS & Gyro");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  Serial.println("LCD Initialized.");
  delay(1000);
  
  // Initialize MPU-6050
  if (!mpu.begin(0x68)) { // Check for MPU at address 0x68
    Serial.println("Failed to find MPU6050 chip");
    lcd.clear();
    lcd.print("MPU-6050");
    lcd.setCursor(0, 1);
    lcd.print("Not Found!");
    while (1) { delay(10); } // Stop if MPU not found
  }
  Serial.println("MPU6050 Found!");
  // Set MPU-6050 default ranges (optional)
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(1000);

  // Start GPS Serial
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("GPS Serial initialized.");

  // Setup Buzzer
  pinMode(BUZZER_PIN, OUTPUT);

  // Check for GPS Module communication
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("GPS Module:");
  lcd.setCursor(0, 1);
  lcd.print("Searching...");
  delay(500);
  unsigned long startTime = millis();
  while (millis() - startTime < 3000) {
    if (gpsSerial.available() > 0) {
      gpsModuleFound = true;
      break;
    }
    delay(100);
  }

  if (!gpsModuleFound) {
    lcd.setCursor(0, 1);
    lcd.print("Not Found!      "); // Clear line
    Serial.println("Warning: GPS module not responding.");
  } else {
    Serial.println("GPS module detected.");
  }
  delay(2000);

  lcd.clear();
  lcd.print("System Ready");
  lcd.setCursor(0, 1);
  lcd.print("Waiting for Fix..");
  Serial.println("Setup complete.");
}

//======================================================================
//  LOOP FUNCTION - Runs repeatedly
//======================================================================
void loop() {
  // Read GPS data when available
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      updateDisplay(); // Update the screen with new data
    }
  }

  // If no GPS data is received after a while, show searching status
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate > 2000) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("GPS Searching...");
      lcd.setCursor(0, 1);
      lcd.print("Waiting for Fix");
      lastUpdate = millis();
    }
  }
}

//======================================================================
//  HELPER FUNCTIONS
//======================================================================

/**
 * @brief Reads all sensors and displays the information on the LCD.
 */
void updateDisplay() {
  lcd.clear();

  // --- LINE 1: GPS Coordinates ---
  lcd.setCursor(0, 0);
  if (gps.location.isValid()) {
    // Format coordinates to 4 decimal places, separated by a semicolon
    String lat_str = String(gps.location.lat(), 4);
    String lng_str = String(gps.location.lng(), 4);
    lcd.print(lat_str + ";" + lng_str);

    // Play melody on first valid fix with enough satellites
    if (gps.satellites.value() >= 4 && !melodyPlayed) {
      Serial.println("GPS Fix Acquired! Playing melody.");
      playMelody();
      melodyPlayed = true;
    }
  } else {
    lcd.print("No GPS Fix");
  }

  // --- LINE 2: Gyroscope Data ---
  lcd.setCursor(0, 1);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Get new sensor readings

  // Format the gyro output as G:X;Y;Z
  lcd.print("G:");
  lcd.print((int)g.gyro.x); // Display as integer for simplicity and space
  lcd.print(";");
  lcd.print((int)g.gyro.y);
  lcd.print(";");
  lcd.print((int)g.gyro.z);

  // Print detailed data to Serial Monitor for debugging
  Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
  Serial.print(" Lng: "); Serial.print(gps.location.lng(), 6);
  Serial.print(" | Gyro X: "); Serial.print(g.gyro.x);
  Serial.print(" Y: "); Serial.print(g.gyro.y);
  Serial.print(" Z: "); Serial.println(g.gyro.z);
}

/**
 * @brief Plays the Pirates of the Caribbean melody on the buzzer.
 */
void playMelody() {
  int melodyLength = sizeof(melody) / sizeof(melody[0]);
  for (int thisNote = 0; thisNote < melodyLength; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(BUZZER_PIN, melody[thisNote], noteDuration);
    delay(noteDuration + 50); // Add a short pause between notes
  }
  noTone(BUZZER_PIN);
}