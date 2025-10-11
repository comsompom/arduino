// ================================================================= //
//                    GYRO SELF-CALIBRATION TEST                    //
//                      MPU-6050 (GY-521) Module                    //
// ================================================================= //
// This sketch combines the direct I2C approach from gy521_test.ino
// with the calibration method from auto_gps_flight.ino
// ================================================================= //

#include "Wire.h" // I2C communication library

// --- MPU-6050 Configuration ---
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050
byte gyro_type = 0; // 0 = not found, 1 = MPU-6050
byte gyro_address = 0x68; // Detected gyro address
byte clockspeed_ok = 0; // I2C clock speed verification

// --- Raw sensor data variables ---
int16_t accelerometer_x, accelerometer_y, accelerometer_z; // Accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // Gyro raw data
int16_t temperature; // Temperature data

// --- Calibration variables ---
float gyro_x_offset = 0, gyro_y_offset = 0, gyro_z_offset = 0; // Calibration offsets
boolean calibration_complete = false;

// --- Angle calculation variables ---
float angle_pitch = 0, angle_roll = 0, angle_yaw = 0; // Calculated angles
float acc_total_vector; // Total accelerometer vector
float angle_pitch_acc, angle_roll_acc; // Accelerometer-based angles

// --- Timing variables ---
unsigned long loop_timer;
const unsigned long LOOP_TIME = 4000; // 4000 microseconds = 250Hz

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
  Serial.begin(57600);
  Serial.println("=== MPU-6050 Gyro Self-Calibration Test ===");
  Serial.println("Initializing...");
  
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
    while(1) { delay(1000); }
  }
  
  // Configure detected gyro
  configureGyro();
  
  Serial.println("MPU-6050 initialized successfully!");
  Serial.println("Starting gyro calibration...");
  Serial.println("Keep the sensor STILL during calibration!");
  
  // Perform gyro calibration
  calibrateGyro();
  
  Serial.println("Calibration complete!");
  Serial.print("Gyro offsets - X: "); Serial.print(gyro_x_offset);
  Serial.print(", Y: "); Serial.print(gyro_y_offset);
  Serial.print(", Z: "); Serial.println(gyro_z_offset);
  Serial.println("Starting angle monitoring...");
  Serial.println("==========================================");
  
  // Set initial loop timer
  loop_timer = micros();
}

// ================================================================= //
//                             MAIN LOOP                             //
// ================================================================= //
void loop() {
  // Read sensor data
  readMPU6050();
  
  // Apply calibration offsets
  gyro_x -= gyro_x_offset;
  gyro_y -= gyro_y_offset;
  gyro_z -= gyro_z_offset;
  
  // Calculate angles
  calculateAngles();
  
  // Display angle information
  displayAngleInfo();
  
  // Maintain loop timing (250Hz)
  while(micros() - loop_timer < LOOP_TIME);
  loop_timer = micros();
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
    
    // Verify PWR_MGMT_1 register
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, (uint8_t)1);
    while(Wire.available() < 1);
    byte reg_value = Wire.read();
    Serial.print("PWR_MGMT_1 register: 0x");
    Serial.println(reg_value, HEX);
    
    // Set gyro range to ±500°/s
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(0x08); // 500°/s range
    Wire.endTransmission();
    
    // Verify GYRO_CONFIG register
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, (uint8_t)1);
    while(Wire.available() < 1);
    reg_value = Wire.read();
    Serial.print("GYRO_CONFIG register: 0x");
    Serial.println(reg_value, HEX);
    
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
 * Read raw data from MPU-6050 (enhanced version from gyro_setup.ino)
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
 * Calculate angles from gyro and accelerometer data
 */
void calculateAngles() {
  // Convert gyro data to degrees per second (500°/s range = 65.5 LSB/°/s)
  float gyro_pitch_rate = gyro_y / 65.5;
  float gyro_roll_rate = gyro_x / 65.5;
  float gyro_yaw_rate = gyro_z / 65.5;
  
  // Calculate time delta (assuming 250Hz = 4000μs loop time)
  float dt = 0.004; // 4ms = 0.004 seconds
  
  // Integrate gyro rates to get angles
  angle_pitch += gyro_pitch_rate * dt;
  angle_roll += gyro_roll_rate * dt;
  angle_yaw += gyro_yaw_rate * dt;
  
  // Calculate accelerometer-based angles
  acc_total_vector = sqrt((accelerometer_x*accelerometer_x) + 
                         (accelerometer_y*accelerometer_y) + 
                         (accelerometer_z*accelerometer_z));
  
  if(abs(accelerometer_y) < acc_total_vector) {
    angle_pitch_acc = asin((float)accelerometer_y/acc_total_vector) * 57.296;
  }
  if(abs(accelerometer_x) < acc_total_vector) {
    angle_roll_acc = asin((float)accelerometer_x/acc_total_vector) * -57.296;
  }
  
  // Complementary filter to correct gyro drift with accelerometer
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;
}

/**
 * Display angle information in a user-friendly format
 */
void displayAngleInfo() {
  // Clear screen (for serial monitor)
  Serial.println("\n=== ANGLE MONITORING ===");
  
  // Display raw sensor data
  Serial.print("Raw Gyro - X: "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | Y: "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | Z: "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  
  // Display calculated angles
  Serial.print("Calculated Angles:");
  Serial.print(" | Pitch: "); Serial.print(angle_pitch, 2);
  Serial.print("° | Roll: "); Serial.print(angle_roll, 2);
  Serial.print("° | Yaw: "); Serial.print(angle_yaw, 2);
  Serial.println("°");
  
  // Display directional information
  Serial.println("--- Directional Analysis ---");
  
  // Front/Back (Pitch)
  if (angle_pitch > 5) {
    Serial.print("FRONT TILT: "); Serial.print(angle_pitch, 1); Serial.println("°");
  } else if (angle_pitch < -5) {
    Serial.print("BACK TILT: "); Serial.print(abs(angle_pitch), 1); Serial.println("°");
  } else {
    Serial.println("FRONT/BACK: Level");
  }
  
  // Left/Right (Roll)
  if (angle_roll > 5) {
    Serial.print("RIGHT TILT: "); Serial.print(angle_roll, 1); Serial.println("°");
  } else if (angle_roll < -5) {
    Serial.print("LEFT TILT: "); Serial.print(abs(angle_roll), 1); Serial.println("°");
  } else {
    Serial.println("LEFT/RIGHT: Level");
  }
  
  // Rotation (Yaw)
  if (angle_yaw > 5) {
    Serial.print("CLOCKWISE ROTATION: "); Serial.print(angle_yaw, 1); Serial.println("°");
  } else if (angle_yaw < -5) {
    Serial.print("COUNTER-CLOCKWISE ROTATION: "); Serial.print(abs(angle_yaw), 1); Serial.println("°");
  } else {
    Serial.println("ROTATION: Stable");
  }
  
  // Display accelerometer-based angles for comparison
  Serial.print("Accelerometer Angles - Pitch: "); Serial.print(angle_pitch_acc, 2);
  Serial.print("° | Roll: "); Serial.print(angle_roll_acc, 2); Serial.println("°");
  
  // Display temperature
  Serial.print("Temperature: "); Serial.print(temperature/340.00+36.53, 1); Serial.println("°C");
  
  Serial.println("==========================");
  
  // Small delay for readability
  delay(100);
}
