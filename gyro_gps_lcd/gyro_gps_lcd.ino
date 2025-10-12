// ================================================================= //
//                    GYRO + GPS + LCD MONITOR                      //
//                      MPU-6050 (GY-521) + GPS + LCD               //
// ================================================================= //
// This sketch combines:
// - Gyro functionality from gyro_self.ino
// - GPS functionality from gps_lcd.ino  
// - LCD display for both gyro and GPS data
// ================================================================= //

#include "Wire.h" // I2C communication library
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>

// --- MPU-6050 Configuration ---
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
byte gyro_type = 0; // 0 = not found, 1 = MPU-6050
byte gyro_address = 0x68; // Detected gyro address
byte clockspeed_ok = 0; // I2C clock speed verification

// --- GPS Configuration ---
SoftwareSerial gpsSerial(4, 3); // RX, TX (connect GPS TX to Arduino pin 4, GPS RX to Arduino pin 3)
TinyGPSPlus gps; // The TinyGPS++ object

// --- LCD Configuration ---
LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C LCD at address 0x27, 20x4 display

// --- Raw sensor data variables ---
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // Accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // Gyro raw data
int16_t temperature; // Temperature data

// --- Calibration variables ---
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; // Calibration offsets
boolean calibration_complete = false;

// --- GPS variables ---
double gps_latitude = 0.0;
double gps_longitude = 0.0;
double gps_altitude = 0.0;
int gps_satellites = 0;
boolean gps_data_valid = false;

// --- Timing variables ---
unsigned long last_display_update = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // Update display every 500ms

// --- Utility function for string conversion ---
char tmp_str[7]; // temporary variable used in convert function
char* convert_int16_to_str(int16_t i) { // converts int16 to string
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

// ================================================================= //
//                              SETUP                                //
// ================================================================= //
void setup() {
  // Initialize Serial communication
  Serial.begin(57600);
  Serial.println("=== GYRO + GPS + LCD MONITOR ===");
  Serial.println("Initializing all systems...");
  
  // Initialize LCD first for user feedback
  initializeLCD();
  
  // Initialize GPS communication
  initializeGPS();
  
  // Initialize I2C and detect gyro
  initializeGyro();
  
  // Perform gyro calibration
  performGyroCalibration();
  
  // System ready
  showSystemReady();
  
  // Initialize display update timer
  last_display_update = millis();
  
  Serial.println("=== ALL SYSTEMS INITIALIZED ===");
  Serial.println("Starting continuous monitoring...");
}

// ================================================================= //
//                             MAIN LOOP                             //
// ================================================================= //
void loop() {
  // Read GPS data using TinyGPS++ library
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // A complete NMEA sentence has been processed
      if (gps.location.isValid()) {
        gps_latitude = gps.location.lat();
        gps_longitude = gps.location.lng();
        gps_data_valid = true;
        
        if (gps.altitude.isValid()) {
          gps_altitude = gps.altitude.meters();
        }
        
        if (gps.satellites.isValid()) {
          gps_satellites = gps.satellites.value();
        }
      } else {
        gps_data_valid = false;
      }
    }
  }
  
  // Read gyro sensor data
  readMPU6050();
  
  // Apply calibration offsets
  gyro_x -= gyro_x_offset;
  gyro_y -= gyro_y_offset;
  gyro_z -= gyro_z_offset;
  
  // Update LCD display at regular intervals
  if(millis() - last_display_update >= DISPLAY_UPDATE_INTERVAL) {
    updateLCD();
    last_display_update = millis();
  }
}

// ================================================================= //
//                    INITIALIZATION FUNCTIONS                       //
// ================================================================= //

/**
 * Initialize LCD display
 */
void initializeLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");
  lcd.setCursor(0, 1);
  lcd.print("Please wait...");
  Serial.println("LCD initialized");
}

/**
 * Initialize GPS communication
 */
void initializeGPS() {
  gpsSerial.begin(9600);
  Serial.println("GPS communication initialized");
}

/**
 * Initialize I2C and detect gyro sensor
 */
void initializeGyro() {
  // Initialize I2C communication
  Wire.begin();
  
  // Set I2C clock speed to 400kHz
  TWBR = 12;
  
  #if F_CPU == 16000000L // If the clock speed is 16MHz
    clockspeed_ok = 1;
  #endif
  
  if(TWBR == 12 && clockspeed_ok) {
    Serial.println("I2C clock speed set to 400kHz");
  } else {
    Serial.println("I2C clock speed setup failed!");
  }
  
  // Search for and detect gyro
  searchAndDetectGyro();
  
  if(gyro_type == 0) {
    Serial.println("No gyro found! Exiting...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("GYRO NOT FOUND!");
    lcd.setCursor(0, 1);
    lcd.print("Check connections");
    while(1) { delay(1000); }
  }
  
  // Configure detected gyro
  configureGyro();
  Serial.println("MPU-6050 initialized successfully!");
}

/**
 * Perform gyro calibration
 */
void performGyroCalibration() {
  Serial.println("Starting gyro calibration...");
  Serial.println("Keep the sensor STILL during calibration!");
  
  // Update LCD with calibration message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating Gyro");
  lcd.setCursor(0, 1);
  lcd.print("Keep still...");
  
  // Perform gyro calibration
  calibrateGyro();
  
  Serial.println("Calibration complete!");
  Serial.print("Gyro offsets - X: "); Serial.print(gyro_x_offset);
  Serial.print(", Y: "); Serial.print(gyro_y_offset);
  Serial.print(", Z: "); Serial.println(gyro_z_offset);
}

/**
 * Show system ready message
 */
void showSystemReady() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Monitoring...");
  delay(2000);
}

// ================================================================= //
//                        HELPER FUNCTIONS                           //
// ================================================================= //

/**
 * Search for and detect gyro sensor
 */
void searchAndDetectGyro() {
  Serial.println("Searching for MPU-6050...");
  
  // Search for MPU-6050 on address 0x68
  if(searchGyro(0x68, 0x75) == 0x68) {
    Serial.println("MPU-6050 found on address 0x68");
    gyro_type = 1;
    gyro_address = 0x68;
    return;
  }
  
  // Search for MPU-6050 on address 0x69
  if(searchGyro(0x69, 0x75) == 0x68) {
    Serial.println("MPU-6050 found on address 0x69");
    gyro_type = 1;
    gyro_address = 0x69;
    return;
  }
  
  Serial.println("MPU-6050 not found!");
}

/**
 * Search for gyro and check the Who_am_I register
 */
byte searchGyro(uint8_t gyro_addr, uint8_t who_am_i) {
  Wire.beginTransmission(gyro_addr);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_addr, (uint8_t)1);
  
  unsigned long timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  
  if(Wire.available() >= 1) {
    return Wire.read();
  }
  return 0;
}

/**
 * Configure detected gyro sensor settings
 */
void configureGyro() {
  if(gyro_type == 1) { // MPU-6050
    Serial.println("Configuring MPU-6050...");
    
    // Wake up the MPU-6050
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0x00); // Set to zero to turn on the gyro
    Wire.endTransmission();
    
    // Set gyro range to ±500°/s
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0x08); // 500°/s range
    Wire.endTransmission();
    
    // Set accelerometer range to ±8g
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(0x10); // ±8g range
    Wire.endTransmission();
    
    // Set digital low pass filter
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A); // CONFIG register
    Wire.write(0x03); // ~43Hz filter
    Wire.endTransmission();
    
    Serial.println("MPU-6050 configuration complete!");
  }
}

/**
 * Read raw data from MPU-6050
 */
void readMPU6050() {
  if(gyro_type == 1) { // MPU-6050
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); // keep connection active
    Wire.requestFrom(gyro_address, (uint8_t)14); // request 14 bytes

    // Read accelerometer data
    accelerometer_x = Wire.read()<<8 | Wire.read(); // 0x3B & 0x3C
    accelerometer_y = Wire.read()<<8 | Wire.read(); // 0x3D & 0x3E
    accelerometer_z = Wire.read()<<8 | Wire.read(); // 0x3F & 0x40
    
    // Read temperature
    temperature = Wire.read()<<8 | Wire.read(); // 0x41 & 0x42
    
    // Read gyro data
    gyro_x = Wire.read()<<8 | Wire.read(); // 0x43 & 0x44
    gyro_y = Wire.read()<<8 | Wire.read(); // 0x45 & 0x46
    gyro_z = Wire.read()<<8 | Wire.read(); // 0x47 & 0x48
  }
}


/**
 * Calibrate gyro by taking multiple samples while stationary
 */
void calibrateGyro() {
  const int CALIBRATION_SAMPLES = 2000; // Number of samples for calibration
  float sum_x = 0, sum_y = 0, sum_z = 0;
  
  Serial.print("Calibrating with ");
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(" samples...");
  
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    readMPU6050();
    
    sum_x += gyro_x;
    sum_y += gyro_y;
    sum_z += gyro_z;
    
    // Show progress every 200 samples
    if (i % 200 == 0) {
      Serial.print("Progress: ");
      Serial.print((i * 100) / CALIBRATION_SAMPLES);
      Serial.println("%");
    }
    
    delay(3); // Small delay between readings
  }
  
  // Calculate average offsets
  gyro_x_offset = sum_x / CALIBRATION_SAMPLES;
  gyro_y_offset = sum_y / CALIBRATION_SAMPLES;
  gyro_z_offset = sum_z / CALIBRATION_SAMPLES;
  
  calibration_complete = true;
}

/**
 * Update LCD display with gyro and GPS data
 */
void updateLCD() {
  lcd.clear();
  
  // First line: Gyro X, Y, Z values
  lcd.setCursor(0, 0);
  lcd.print("X:");
  lcd.print(gyro_x);
  lcd.print(" Y:");
  lcd.print(gyro_y);
  lcd.print(" Z:");
  lcd.print(gyro_z);
  
  // Second line: GPS coordinates (rounded to 3 decimal places)
  lcd.setCursor(0, 1);
  if (gps_data_valid) {
    lcd.print(gps_latitude, 3);
    lcd.print("_");
    lcd.print(gps_longitude, 3);
  } else {
    lcd.print("GPS: No signal");
  }
  
  // Third line: GPS altitude and satellites
  lcd.setCursor(0, 2);
  if (gps_data_valid) {
    lcd.print("Alt:");
    lcd.print(gps_altitude, 1);
    lcd.print("m Sat:");
    lcd.print(gps_satellites);
  } else {
    lcd.print("Waiting for GPS...");
  }
  
  // Fourth line: Temperature and Status
  lcd.setCursor(0, 3);
  lcd.print("T:");
  lcd.print(temperature/340.00+36.53, 1);
  lcd.print("C ");
  if (calibration_complete && gps_data_valid) {
    lcd.print("Ready");
  } else if (calibration_complete) {
    lcd.print("GPS Wait");
  } else {
    lcd.print("Calibrating");
  }
}
