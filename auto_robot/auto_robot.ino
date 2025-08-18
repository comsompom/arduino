//======================================================================
// LIBRARIES
//======================================================================
#include <Wire.h>
#include <AFMotor.h>

//======================================================================
// PIN & OBJECT DEFINITIONS
//======================================================================
// Motor control using AFMotor library (working solution)
AF_DCMotor leftMotor(1);  // Motor 1
AF_DCMotor rightMotor(2); // Motor 2

// GY-65 (BMP180) I2C address
#define BMP180_ADDR_DEFAULT 0x77
byte BMP180_ADDR = BMP180_ADDR_DEFAULT; // Will be updated during initialization

// Sonar Pins
const int FWD_TRIG_PIN = 30;
const int FWD_ECHO_PIN = 31;
const int GND_TRIG_PIN = 32;
const int GND_ECHO_PIN = 33;

// Buzzer Pin
const int BUZZER_PIN = 22;

//======================================================================
// CONSTANTS & CALIBRATION
//======================================================================
// *** YOU MUST CALIBRATE THIS VALUE! ***
// Measure how many cm your robot moves in 1 second at a speed of 150.
// Then update this value. Example: if it moves 25cm, set this to 25.0.
const float ROBOT_SPEED_CM_PER_S = 25.0; 

// Motor speed (0-255)
const int MOTOR_SPEED = 150;
const int TURN_SPEED = 130;

// Obstacle detection thresholds (in cm)
const int OBSTACLE_THRESHOLD_CM = 20; // If something is closer than this, it's an obstacle.
const int CLIFF_THRESHOLD_CM = 15;    // If the ground is further than this, it's a cliff/drop.

// Distance measurement variables
float initialDistance = 0;
float currentDistance = 0;

//======================================================================
// SETUP FUNCTION - Runs once at the beginning
//======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Robot starting up...");

  // --- Initial Setup & Beep ---
  pinMode(BUZZER_PIN, OUTPUT);
  beep(1, 250); // Beep once on power-up

  // --- Initialize I2C for GY-65 ---
  Wire.begin();
  Serial.println("I2C initialized for GY-65 sensor.");

  // --- Scan I2C bus to see what devices are connected ---
  Serial.println("Scanning I2C bus for connected devices...");
  scanI2C();

  // --- Motor Pin Setup ---
  // AFMotor library handles pin configuration internally
  Serial.println("Motor pins configured.");
  
  // Initialize motors
  leftMotor.setSpeed(0);
  rightMotor.setSpeed(0);
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
  Serial.println("Motors initialized and stopped.");
  
  // Print motor shield information
  printMotorShieldInfo();
  
  // Help identify motor shield
  identifyMotorShield();

  // --- Test Motors ---
  Serial.println("Testing motors...");
  manualMotorTest(); // Use manual test instead

  // --- Sonar Pin Setup ---
  pinMode(FWD_TRIG_PIN, OUTPUT);
  pinMode(FWD_ECHO_PIN, INPUT);
  pinMode(GND_TRIG_PIN, OUTPUT);
  pinMode(GND_ECHO_PIN, INPUT);
  Serial.println("Sonar pins configured.");

  // --- GY-65 (BMP180) Setup & Calibration ---
  Serial.println("Attempting to connect to GY-65 (BMP180)...");
  
  // Test I2C connection first
  if (!checkI2CConnection()) {
    Serial.println("I2C connection test failed. Please check:");
    Serial.println("1. SDA and SCL connections (SDA=pin 20, SCL=pin 21 on Mega 2560)");
    Serial.println("2. Power supply to GY-65 sensor");
    Serial.println("3. Pull-up resistors (if needed)");
    Serial.println("4. No short circuits on I2C lines");
    while (1);
  }
  
  if (!initializeBMP180()) {
    Serial.println("Could not find GY-65 (BMP180). Halting.");
    Serial.println("Please check GY-65 connections and try again.");
    while (1);
  }
  Serial.println("GY-65 (BMP180) connection successful");
  
  calibrateDistance();

  // --- Post-setup Beeps ---
  beep(2, 100); // Beep twice to indicate setup is complete

  Serial.println("\nSetup complete. Ready to move.");
  delay(1000); // Wait a moment before starting the main sequence
}

//======================================================================
// MAIN LOOP - Contains the robot's movement sequence
//======================================================================
void loop() {
  // This is the main sequence as requested.
  Serial.println("Step 1: Moving forward 2 meters...");
  moveForwardWithObstacleCheck(200); // 200 cm = 2 meters

  Serial.println("Step 2: Turning right 90 degrees...");
  turnRight(90);

  Serial.println("Step 3: Moving forward 50 cm...");
  moveForwardWithObstacleCheck(50);

  Serial.println("Step 4: Turning right 90 degrees...");
  turnRight(90);
  
  Serial.println("Step 5: Moving forward 2 meters...");
  moveForwardWithObstacleCheck(200);

  Serial.println("\n--- Sequence Complete. Halting. ---");
  stopMotors();
  while(1); // Stop the program here
}

//======================================================================
// MOVEMENT & SENSOR FUNCTIONS
//======================================================================

/**
 * Moves the robot forward for a given distance, continuously checking for obstacles.
 * @param distanceCm The distance to travel in centimeters.
 */
void moveForwardWithObstacleCheck(float distanceCm) {
  // Calculate the time required to travel the distance based on the calibrated speed
  float durationSeconds = distanceCm / ROBOT_SPEED_CM_PER_S;
  long durationMillis = durationSeconds * 1000;

  unsigned long startTime = millis();
  
  while (millis() - startTime < durationMillis) {
    if (checkForObstacles()) {
      Serial.println("Obstacle detected! Attempting to avoid...");
      stopMotors();
      if (avoidObstacle()) {
        Serial.println("Obstacle avoided, continuing...");
        // Recalculate remaining time and continue
        unsigned long elapsedTime = millis() - startTime;
        long remainingTime = durationMillis - elapsedTime;
        if (remainingTime > 0) {
          startTime = millis() - remainingTime;
          continue;
        }
      } else {
        Serial.println("Could not avoid obstacle. Stopping movement.");
        return; // Exit the function
      }
    }
    
    // If no obstacle, keep moving forward
    setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
    delay(20); // Small delay to prevent overwhelming the loop
  }
  
  // Stop after the time has elapsed
  stopMotors();
}

/**
 * Turns the robot right by a specified angle using time-based turning.
 * @param targetAngle The angle in degrees to turn.
 */
void turnRight(float targetAngle) {
  // Calculate turn time based on angle (empirical calibration needed)
  // Assuming 90 degrees takes about 1.5 seconds at TURN_SPEED
  float turnTimeSeconds = (targetAngle / 90.0) * 1.5;
  long turnTimeMillis = turnTimeSeconds * 1000;
  
  // Start turning: right motor backward, left motor forward
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  
  delay(turnTimeMillis);
  
  stopMotors();
  delay(200); // Pause after turn
}

/**
 * Attempts to avoid obstacles by turning left or right.
 * @return True if avoidance was successful, false otherwise.
 */
bool avoidObstacle() {
  // Check which side has more space
  long leftDistance = checkLeftSide();
  long rightDistance = checkRightSide();
  
  Serial.print("Left distance: "); Serial.print(leftDistance); Serial.println(" cm");
  Serial.print("Right distance: "); Serial.print(rightDistance); Serial.println(" cm");
  
  if (leftDistance > rightDistance && leftDistance > OBSTACLE_THRESHOLD_CM) {
    // Turn left to avoid
    Serial.println("Turning left to avoid obstacle...");
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    delay(1000); // Turn for 1 second
    stopMotors();
    return true;
  } else if (rightDistance > OBSTACLE_THRESHOLD_CM) {
    // Turn right to avoid
    Serial.println("Turning right to avoid obstacle...");
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
    delay(1000); // Turn for 1 second
    stopMotors();
    return true;
  }
  
  return false; // No clear path found
}

/**
 * Checks the left side distance using the forward sonar.
 * @return Distance in centimeters.
 */
long checkLeftSide() {
  // For simplicity, using the forward sonar
  // In a real implementation, you might have a dedicated left sonar
  return readSonar(FWD_TRIG_PIN, FWD_ECHO_PIN);
}

/**
 * Checks the right side distance using the forward sonar.
 * @return Distance in centimeters.
 */
long checkRightSide() {
  // For simplicity, using the forward sonar
  // In a real implementation, you might have a dedicated right sonar
  return readSonar(FWD_TRIG_PIN, FWD_ECHO_PIN);
}

/**
 * Reads a sonar sensor and returns the distance in cm.
 * @param trigPin The trigger pin of the sonar.
 * @param echoPin The echo pin of the sonar.
 * @return Distance in centimeters.
 */
long readSonar(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2; // Calculate distance in cm
}

/**
 * Checks both sonars for obstacles or cliffs.
 * @return True if an obstacle is found, false otherwise.
 */
bool checkForObstacles() {
  long forwardDistance = readSonar(FWD_TRIG_PIN, FWD_ECHO_PIN);
  long groundDistance = readSonar(GND_TRIG_PIN, GND_ECHO_PIN);

  if (forwardDistance < OBSTACLE_THRESHOLD_CM && forwardDistance > 0) {
    Serial.print("! Forward Obstacle at ");
    Serial.print(forwardDistance);
    Serial.println(" cm");
    return true;
  }

  if (groundDistance > CLIFF_THRESHOLD_CM) {
    Serial.print("! Cliff Detected! Ground distance: ");
    Serial.print(groundDistance);
    Serial.println(" cm");
    return true;
  }

  return false;
}

//======================================================================
// GY-65 (BMP180) FUNCTIONS
//======================================================================

/**
 * Initializes the BMP180 sensor.
 * @return True if successful, false otherwise.
 */
bool initializeBMP180() {
  // Try both possible BMP180 addresses
  byte addresses[] = {0x77, 0x76}; // Common BMP180 addresses
  
  for (int i = 0; i < 2; i++) {
    byte addr = addresses[i];
    Serial.print("Trying BMP180 at address 0x");
    Serial.println(addr, HEX);
    
    Wire.beginTransmission(addr);
    Wire.write(0xD0); // Chip ID register
    byte error = Wire.endTransmission(false);
    
    if (error != 0) {
      Serial.print("Error ");
      Serial.print(error);
      Serial.print(" when trying to communicate with address 0x");
      Serial.println(addr, HEX);
      continue;
    }
    
    Wire.requestFrom((uint8_t)addr, (uint8_t)1, (uint8_t)true);
    if (Wire.available()) {
      byte chipId = Wire.read();
      Serial.print("Chip ID: 0x");
      Serial.println(chipId, HEX);
      
      if (chipId == 0x55) { // BMP180 chip ID
        Serial.print("BMP180 found at address 0x");
        Serial.println(addr, HEX);
        // Update the global address
        BMP180_ADDR = addr;
        return true;
      } else {
        Serial.print("Wrong chip ID: 0x");
        Serial.println(chipId, HEX);
      }
    } else {
      Serial.println("No response from device");
    }
  }
  
  Serial.println("BMP180 not found on any expected address");
  return false;
}

/**
 * Reads temperature from BMP180.
 * @return Temperature in degrees Celsius.
 */
float readTemperature() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4); // Control register
  Wire.write(0x2E); // Start temperature measurement
  Wire.endTransmission();
  delay(5); // Wait for measurement
  
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6); // Data register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)2, (uint8_t)true);
  
  int rawTemp = Wire.read() << 8 | Wire.read();
  float temp = (float)rawTemp / 10.0;
  return temp;
}

/**
 * Reads pressure from BMP180.
 * @return Pressure in Pa.
 */
long readPressure() {
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF4); // Control register
  Wire.write(0x34); // Start pressure measurement
  Wire.endTransmission();
  delay(26); // Wait for measurement
  
  Wire.beginTransmission(BMP180_ADDR);
  Wire.write(0xF6); // Data register
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)BMP180_ADDR, (uint8_t)3, (uint8_t)true);
  
  long pressure = Wire.read() << 16 | Wire.read() << 8 | Wire.read();
  pressure >>= 8;
  return pressure;
}

/**
 * Calculates altitude from pressure.
 * @param pressure Pressure in Pa.
 * @return Altitude in meters.
 */
float calculateAltitude(long pressure) {
  float altitude = 44330 * (1 - pow(pressure / 101325.0, 0.1903));
  return altitude;
}

/**
 * Calibrates the distance measurement using the GY-65.
 */
void calibrateDistance() {
  Serial.println("Calibrating distance measurement with GY-65...");
  
  // Take multiple readings and average them
  float totalAltitude = 0;
  const int samples = 10;
  
  for (int i = 0; i < samples; i++) {
    long pressure = readPressure();
    float altitude = calculateAltitude(pressure);
    totalAltitude += altitude;
    delay(100);
  }
  
  initialDistance = totalAltitude / samples;
  currentDistance = initialDistance;
  
  Serial.print("Initial distance calibrated: ");
  Serial.print(initialDistance);
  Serial.println(" meters");
}

/**
 * Updates the current distance measurement.
 */
void updateDistance() {
  long pressure = readPressure();
  float altitude = calculateAltitude(pressure);
  currentDistance = altitude;
}

//======================================================================
// UTILITY & HELPER FUNCTIONS
//======================================================================

/**
 * Creates beeps with the buzzer.
 * @param count Number of beeps.
 * @param durationMs Duration of each beep in milliseconds.
 */
void beep(int count, int durationMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(durationMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (count > 1) {
      delay(durationMs);
    }
  }
}

/**
 * Controls both motors at a given speed.
 * @param leftSpeed Speed for the left motor (-255 to 255).
 * @param rightSpeed Speed for the right motor (-255 to 255).
 */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  Serial.print("Setting motor speeds - Left: ");
  Serial.print(leftSpeed);
  Serial.print(", Right: ");
  Serial.println(rightSpeed);
  
  // Control Left Motor
  if (leftSpeed > 0) {
    leftMotor.run(FORWARD);
    leftMotor.setSpeed(leftSpeed);
    Serial.println("Left motor: FORWARD");
  } else if (leftSpeed < 0) {
    leftMotor.run(BACKWARD);
    leftMotor.setSpeed(-leftSpeed);
    Serial.println("Left motor: BACKWARD");
  } else {
    leftMotor.run(RELEASE);
    Serial.println("Left motor: STOP");
  }

  // Control Right Motor
  if (rightSpeed > 0) {
    rightMotor.run(FORWARD);
    rightMotor.setSpeed(rightSpeed);
    Serial.println("Right motor: FORWARD");
  } else if (rightSpeed < 0) {
    rightMotor.run(BACKWARD);
    rightMotor.setSpeed(-rightSpeed);
    Serial.println("Right motor: BACKWARD");
  } else {
    rightMotor.run(RELEASE);
    Serial.println("Right motor: STOP");
  }
}

/**
 * Stops both motors.
 */
void stopMotors() {
  leftMotor.run(RELEASE);
  rightMotor.run(RELEASE);
}

/**
 * Scans the I2C bus for connected devices.
 */
void scanI2C() {
  byte error, address;
  int nDevices = 0;
  Serial.println("I2C devices found:");
  for (address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
      nDevices++;
    }
    else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

/**
 * Tests the I2C connection to the GY-65 sensor.
 * @return True if connection is successful, false otherwise.
 */
bool checkI2CConnection() {
  Serial.println("Testing I2C connection to GY-65...");
  Wire.beginTransmission(BMP180_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.print("GY-65 found at address 0x");
    Serial.println(BMP180_ADDR, HEX);
    return true;
  } else {
    Serial.print("GY-65 not found at address 0x");
    Serial.println(BMP180_ADDR, HEX);
    return false;
  }
}

/**
 * Manual motor test function - call this from setup() to test motors manually.
 * Comment out the automatic test and uncomment this line in setup():
 * manualMotorTest();
 */
void manualMotorTest() {
  Serial.println("\n=== MANUAL MOTOR TEST ===");
  Serial.println("Testing basic motor functionality...");
  
  // Test forward movement
  Serial.println("Testing forward movement...");
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(150);
  delay(3000);
  stopMotors();
  delay(1000);
  
  // Test backward movement
  Serial.println("Testing backward movement...");
  leftMotor.run(BACKWARD);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(150);
  rightMotor.setSpeed(150);
  delay(3000);
  stopMotors();
  delay(1000);
  
  // Test turn right
  Serial.println("Testing turn right...");
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(130);
  rightMotor.setSpeed(130);
  delay(2000);
  stopMotors();
  delay(1000);
  
  Serial.println("Manual motor test complete.");
}

/**
 * Provides information about common motor shield pin configurations.
 */
void printMotorShieldInfo() {
  Serial.println("\n=== MOTOR SHIELD CONFIGURATION ===");
  Serial.println("Using AFMotor library for motor control.");
  Serial.println("This library automatically handles pin configuration.");
  Serial.println();
  Serial.println("Motor Configuration:");
  Serial.println("  Left Motor:  AF_DCMotor(1)");
  Serial.println("  Right Motor: AF_DCMotor(2)");
  Serial.println();
  Serial.println("AFMotor library features:");
  Serial.println("  - Automatic pin configuration");
  Serial.println("  - Built-in speed control");
  Serial.println("  - Direction control (FORWARD/BACKWARD/RELEASE)");
  Serial.println("  - Compatible with Adafruit Motor Shield");
  Serial.println("=====================================\n");
}

/**
 * Helps identify the motor shield type.
 */
void identifyMotorShield() {
  Serial.println("\n=== MOTOR SHIELD IDENTIFICATION ===");
  Serial.println("Using AFMotor library for motor control.");
  Serial.println("Testing motor movements...");
  
  // Test forward movement
  Serial.println("Testing forward movement...");
  leftMotor.run(FORWARD);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(MOTOR_SPEED);
  rightMotor.setSpeed(MOTOR_SPEED);
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Test backward movement
  Serial.println("Testing backward movement...");
  leftMotor.run(BACKWARD);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(MOTOR_SPEED);
  rightMotor.setSpeed(MOTOR_SPEED);
  delay(2000);
  stopMotors();
  delay(1000);
  
  // Test turn right
  Serial.println("Testing turn right...");
  leftMotor.run(FORWARD);
  rightMotor.run(BACKWARD);
  leftMotor.setSpeed(TURN_SPEED);
  rightMotor.setSpeed(TURN_SPEED);
  delay(1000);
  stopMotors();
  delay(1000);
  
  // Test turn left
  Serial.println("Testing turn left...");
  leftMotor.run(BACKWARD);
  rightMotor.run(FORWARD);
  leftMotor.setSpeed(TURN_SPEED);
  rightMotor.setSpeed(TURN_SPEED);
  delay(1000);
  stopMotors();
  delay(1000);
  
  Serial.println("=== MOTOR SHIELD IDENTIFICATION COMPLETE ===");
  Serial.println("AFMotor library is working correctly.");
}
