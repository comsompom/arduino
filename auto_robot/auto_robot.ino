//======================================================================
// LIBRARIES
//======================================================================
#include <Wire.h>
#include <AFMotor.h>
#include <AccelStepper.h>

//======================================================================
// PIN & OBJECT DEFINITIONS
//======================================================================
// Motor control using AFMotor library with stepper motors (working solution)
AF_Stepper motor1(200, 1);  // Left motor
AF_Stepper motor2(200, 2);  // Right motor

// Gyro sensor address
#define MPU6050_ADDR 0x68

// Stepper motor control functions - exactly like test_motor.ino
void leftForwardStep() {  
  motor1.onestep(FORWARD, SINGLE);
}
void leftBackwardStep() {  
  motor1.onestep(BACKWARD, SINGLE);
}
void rightForwardStep() {  
  motor2.onestep(FORWARD, SINGLE);
}
void rightBackwardStep() {  
  motor2.onestep(BACKWARD, SINGLE);
}

// Create AccelStepper objects - exactly like test_motor.ino
AccelStepper leftStepper(leftForwardStep, leftBackwardStep);
AccelStepper rightStepper(rightForwardStep, rightBackwardStep);

// GY-65 (BMP180) I2C address
#define BMP180_ADDR_DEFAULT 0x77
byte BMP180_ADDR = BMP180_ADDR_DEFAULT; // Will be updated during initialization

// Sonar Pins
const int FWD_TRIG_PIN = 30;
const int FWD_ECHO_PIN = 31;
const int GND_TRIG_PIN = 32;
const int GND_ECHO_PIN = 33;

// Buzzer Pin
const int BUZZER_PIN = A3; // Changed from digital pin 22 to analog pin 3

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
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Auto Robot Starting...");
  
  // Initialize I2C communication
  Wire.begin();
  
  // Initialize buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  
  // Initialize sonar sensors
  pinMode(FWD_TRIG_PIN, OUTPUT);
  pinMode(FWD_ECHO_PIN, INPUT);
  pinMode(GND_TRIG_PIN, OUTPUT);
  pinMode(GND_ECHO_PIN, INPUT);
  
  // Initialize stepper motors
  leftStepper.setSpeed(50);
  rightStepper.setSpeed(50);
  Serial.println("Stepper motors initialized.");
  
  // Print motor shield information
  printMotorShieldInfo();
  
  // Initial buzzer beep
  beep(1, 250);
  delay(1000);
  
  // Initialize and calibrate GY-65 (BMP180)
  Serial.println("Initializing GY-65 (BMP180)...");
  if (!initializeBMP180()) {
    Serial.println("Could not find GY-65 (BMP180). Halting.");
    while (1) {
      beep(3, 500);
      delay(2000);
    }
  }
  
  // Initialize and calibrate MPU6050 (Gyro)
  Serial.println("Initializing MPU6050 (Gyro)...");
  if (!initializeMPU6050()) {
    Serial.println("Could not find MPU6050 (Gyro). Halting.");
    while (1) {
      beep(3, 500);
      delay(2000);
    }
  }
  
  // Calibrate distance sensor
  Serial.println("Calibrating distance sensor...");
  calibrateDistance();
  
  // Setup completion buzzer beeps
  beep(2, 100);
  
  Serial.println("Setup complete. Robot ready to move!");
  delay(2000);
}

//======================================================================
// MAIN LOOP - Contains the robot's movement sequence
//======================================================================
void loop() {
  // Reset gyro yaw to zero at the start of the sequence
  resetGyroYaw();
  
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
 * Moves the robot forward for a given distance using gyro-based navigation.
 * Continuously monitors gyro data to maintain straight line movement.
 * @param distanceCm The distance to travel in centimeters.
 */
void moveForwardWithObstacleCheck(float distanceCm) {
  Serial.print("Moving forward "); Serial.print(distanceCm); Serial.println(" cm");
  
  // Get initial gyro reading for reference
  float initialYaw = getGyroYaw();
  float currentYaw = initialYaw;
  float yawError = 0;
  
  // Calculate the time required to travel the distance based on the calibrated speed
  float durationSeconds = distanceCm / ROBOT_SPEED_CM_PER_S;
  long durationMillis = durationSeconds * 1000;

  unsigned long startTime = millis();
  unsigned long lastGyroCheck = millis();
  
  while (millis() - startTime < durationMillis) {
    // Check for obstacles every 100ms
    if (millis() - lastGyroCheck > 100) {
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
            // Reset gyro reference after obstacle avoidance
            initialYaw = getGyroYaw();
            continue;
          }
        } else {
          Serial.println("Could not avoid obstacle. Stopping movement.");
          return; // Exit the function
        }
      }
      lastGyroCheck = millis();
    }
    
    // Get current gyro reading and calculate error
    currentYaw = getGyroYaw();
    yawError = currentYaw - initialYaw;
    
    // Apply gyro correction to maintain straight line
    int leftSpeed = MOTOR_SPEED;
    int rightSpeed = MOTOR_SPEED;
    
    // Simple proportional correction
    if (yawError > 2.0) { // Robot turning right, slow down left motor
      leftSpeed = MOTOR_SPEED - (int)(yawError * 5);
      if (leftSpeed < 0) leftSpeed = 0;
    } else if (yawError < -2.0) { // Robot turning left, slow down right motor
      rightSpeed = MOTOR_SPEED - (int)(-yawError * 5);
      if (rightSpeed < 0) rightSpeed = 0;
    }
    
    // Set motor speeds with gyro correction
    setMotorSpeed(leftSpeed, rightSpeed);
    
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
    // Print gyro data for debugging
    Serial.print("Yaw: "); Serial.print(currentYaw, 1);
    Serial.print(" Error: "); Serial.print(yawError, 1);
    Serial.print(" L:"); Serial.print(leftSpeed);
    Serial.print(" R:"); Serial.println(rightSpeed);
    
    delay(50); // Small delay for gyro reading stability
  }
  
  // Stop after the time has elapsed
  stopMotors();
  Serial.println("Forward movement complete.");
}

/**
 * Turns the robot right by a specified angle using gyro-based turning.
 * @param targetAngle The angle in degrees to turn.
 */
void turnRight(float targetAngle) {
  Serial.print("Turning right "); Serial.print(targetAngle); Serial.println(" degrees");
  
  // Get initial gyro reading
  float initialYaw = getGyroYaw();
  float targetYaw = initialYaw + targetAngle;
  float currentYaw = initialYaw;
  
  // Start turning: right motor backward, left motor forward
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  
  while (currentYaw < targetYaw - 1.0) { // Allow 1 degree tolerance
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
    // Get current gyro reading
    currentYaw = getGyroYaw();
    
    // Print gyro data for debugging
    Serial.print("Turning - Current: "); Serial.print(currentYaw, 1);
    Serial.print(" Target: "); Serial.print(targetYaw, 1);
    Serial.print(" Remaining: "); Serial.println(targetYaw - currentYaw, 1);
    
    delay(50);
  }
  
  stopMotors();
  delay(200); // Pause after turn
  Serial.println("Turn complete.");
}

/**
 * Attempts to avoid obstacles by turning left or right using gyro-based navigation.
 * @return True if avoidance was successful, false otherwise.
 */
bool avoidObstacle() {
  // Check which side has more space
  long leftDistance = checkLeftSide();
  long rightDistance = checkRightSide();
  
  Serial.print("Left distance: "); Serial.print(leftDistance); Serial.println(" cm");
  Serial.print("Right distance: "); Serial.print(rightDistance); Serial.println(" cm");
  
  if (leftDistance > rightDistance && leftDistance > OBSTACLE_THRESHOLD_CM) {
    // Turn left to avoid using gyro
    Serial.println("Turning left 45 degrees to avoid obstacle...");
    turnLeft(45);
    return true;
  } else if (rightDistance > OBSTACLE_THRESHOLD_CM) {
    // Turn right to avoid using gyro
    Serial.println("Turning right 45 degrees to avoid obstacle...");
    turnRight(45);
    return true;
  }
  
  return false; // No clear path found
}

/**
 * Turns the robot left by a specified angle using gyro-based turning.
 * @param targetAngle The angle in degrees to turn.
 */
void turnLeft(float targetAngle) {
  Serial.print("Turning left "); Serial.print(targetAngle); Serial.println(" degrees");
  
  // Get initial gyro reading
  float initialYaw = getGyroYaw();
  float targetYaw = initialYaw - targetAngle; // Negative for left turn
  float currentYaw = initialYaw;
  
  // Start turning: left motor backward, right motor forward
  setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  
  while (currentYaw > targetYaw + 1.0) { // Allow 1 degree tolerance
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
    // Get current gyro reading
    currentYaw = getGyroYaw();
    
    // Print gyro data for debugging
    Serial.print("Turning Left - Current: "); Serial.print(currentYaw, 1);
    Serial.print(" Target: "); Serial.print(targetYaw, 1);
    Serial.print(" Remaining: "); Serial.println(currentYaw - targetYaw, 1);
    
    delay(50);
  }
  
  stopMotors();
  delay(200); // Pause after turn
  Serial.println("Left turn complete.");
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
    leftStepper.setSpeed(leftSpeed);
    Serial.println("Left motor: FORWARD");
  } else if (leftSpeed < 0) {
    leftStepper.setSpeed(-leftSpeed);
    Serial.println("Left motor: BACKWARD");
  } else {
    leftStepper.setSpeed(0);
    Serial.println("Left motor: STOP");
  }

  // Control Right Motor
  if (rightSpeed > 0) {
    rightStepper.setSpeed(rightSpeed);
    Serial.println("Right motor: FORWARD");
  } else if (rightSpeed < 0) {
    rightStepper.setSpeed(-rightSpeed);
    Serial.println("Right motor: BACKWARD");
  } else {
    rightStepper.setSpeed(0);
    Serial.println("Right motor: STOP");
  }
}

/**
 * Stops both motors.
 */
void stopMotors() {
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);
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
 * Initializes the MPU6050 (Gyro) sensor.
 * @return True if successful, false otherwise.
 */
bool initializeMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  if (Wire.endTransmission() == 0) {
    Serial.print("MPU6050 found at address 0x");
    Serial.println(MPU6050_ADDR, HEX);
    return true;
  } else {
    Serial.print("MPU6050 not found at address 0x");
    Serial.println(MPU6050_ADDR, HEX);
    return false;
  }
}

/**
 * Gets the current yaw angle from the gyro sensor.
 * @return Current yaw angle in degrees.
 */
float getGyroYaw() {
  // Read gyro data from MPU6050
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
  
  if (Wire.available() >= 14) {
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    
    // Convert gyro reading to degrees per second
    float gyroZ = gz / 131.0; // MPU6050 sensitivity for ±250°/s range
    
    // Simple integration for yaw (in a real application, you'd use a proper filter)
    static float yaw = 0;
    static unsigned long lastTime = 0;
    static bool firstReading = true;
    unsigned long currentTime = millis();
    
    if (firstReading) {
      lastTime = currentTime;
      firstReading = false;
      return 0; // Return 0 for first reading
    }
    
    if (lastTime > 0) {
      float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
      yaw += gyroZ * dt;
    }
    lastTime = currentTime;
    
    return yaw;
  }
  
  return 0; // Return 0 if reading failed
}

/**
 * Resets the gyro yaw to zero.
 */
void resetGyroYaw() {
  Serial.println("Resetting gyro yaw to zero...");
  
  // Reset the static variables in getGyroYaw function
  // This is a simple approach - in a real application you'd use a proper filter
  static float yaw = 0;
  static unsigned long lastTime = 0;
  static bool firstReading = true;
  
  // Reset the static variables
  yaw = 0;
  lastTime = 0;
  firstReading = true;
  
  // Small delay to let gyro stabilize
  delay(100);
  Serial.println("Gyro yaw reset complete.");
}

/**
 * Provides information about motor configuration.
 */
void printMotorShieldInfo() {
  Serial.println("\n=== MOTOR CONFIGURATION ===");
  Serial.println("Using AFMotor library with stepper motors.");
  Serial.println("Motor Configuration:");
  Serial.println("  Left Motor:  AF_Stepper(200, 1)");
  Serial.println("  Right Motor: AF_Stepper(200, 2)");
  Serial.println("=====================================\n");
}

