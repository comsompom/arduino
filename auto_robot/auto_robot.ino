//======================================================================
// LIBRARIES
//======================================================================
#include <Wire.h>
#include <AFMotor.h>

// ADXL345 Accelerometer (GY-291)
#define ADXL345_ADDR 0x53  // ADXL345 I2C address
#define ADXL345_DEVID 0xE5 // Device ID for ADXL345

// Motor definitions - using DC motors like in motor_move.ino
AF_DCMotor motor1(1);  // Left motor on M1
AF_DCMotor motor2(2);  // Right motor on M2

// ADXL345 Accelerometer variables
bool adxl345_found = false;
float adxl345_offset[3] = {0, 0, 0}; // Calibration offsets for X, Y, Z
float adxl345_position[3] = {0, 0, 0}; // Current position (X, Y, Z)
float adxl345_velocity[3] = {0, 0, 0}; // Current velocity (X, Y, Z)
unsigned long last_adxl345_read = 0;
const unsigned long ADXL345_READ_INTERVAL = 50; // Read every 50ms

// Sonar Pins
const int FWD_TRIG_PIN = 30;
const int FWD_ECHO_PIN = 31;
const int GND_TRIG_PIN = 32;
const int GND_ECHO_PIN = 33;

// Buzzer Pin
const int BUZZER_PIN = 13;

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

//======================================================================
// USER-DEFINED MOVEMENT PLAN
//======================================================================
// Define your movement plan here using the following patterns:
// "FWD_XXX" - move forward XXX centimeters
// "BKWD_XXX" - move backward XXX centimeters  
// "LT_XX" - turn left XX degrees
// "RT_XX" - turn right XX degrees
//
// Example movement plan:
// Step 1: Forward 200cm (2 meters)
// Step 2: Right turn 90 degrees
// Step 3: Forward 50cm
// Step 4: Right turn 90 degrees
// Step 5: Forward 200cm (2 meters)

String movementPlan[] = {
  "FWD_200",  // Move forward 200cm (2 meters)
  "RT_90",    // Turn right 90 degrees
  "FWD_50",   // Move forward 50cm
  "RT_90",    // Turn right 90 degrees
  "FWD_200"   // Move forward 200cm (2 meters)
};

const int MOVEMENT_PLAN_SIZE = sizeof(movementPlan) / sizeof(movementPlan[0]);

//======================================================================
// SETUP FUNCTION - Runs once at the beginning
//======================================================================
void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Auto Robot Starting...");
  
  // Simple buzzer test - using working tone code
  noTone(BUZZER_PIN);
  tone(BUZZER_PIN, 494, 500);
  delay(500);
  noTone(BUZZER_PIN);
  
  // Initialize I2C communication
  Wire.begin();
  Serial.println("I2C communication initialized.");
  
  // Initialize and detect ADXL345 accelerometer
  Serial.println("Initializing ADXL345 accelerometer...");
  if (initializeADXL345()) {
    Serial.println("ADXL345 found and initialized successfully!");
    adxl345_found = true;
    
    // Calibrate ADXL345 with 400 attempts
    Serial.println("Calibrating ADXL345 with 400 samples...");
    calibrateADXL345(400);
    Serial.println("ADXL345 calibration complete!");
  } else {
    Serial.println("ADXL345 not found! Continuing without accelerometer...");
    adxl345_found = false;
  }
  
  // Initialize sonar sensors
  pinMode(FWD_TRIG_PIN, OUTPUT);
  pinMode(FWD_ECHO_PIN, INPUT);
  pinMode(GND_TRIG_PIN, OUTPUT);
  pinMode(GND_ECHO_PIN, INPUT);
  Serial.println("Sonar sensors initialized.");
  
  // Initialize stepper motors
  Serial.println("Initializing DC motors...");
  // Set initial speed to 0 and stop motors
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  Serial.println("DC motors initialized.");
  
  // Test DC motors briefly
  Serial.println("Testing DC motors...");
  motor1.setSpeed(100);
  motor2.setSpeed(100);
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  delay(500);
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  Serial.println("DC motor test complete.");
  
  // Print motor shield information
  Serial.println("Printing motor configuration...");
  printMotorShieldInfo();
  
  // Initial buzzer beep using working tone code
  Serial.println("Initial buzzer beep...");
  tone(BUZZER_PIN, 1000, 250);
  delay(1000);
  
  // Setup completion buzzer beeps using working tone code
  Serial.println("Setup completion buzzer...");
  tone(BUZZER_PIN, 1000, 100);
  delay(200);
  tone(BUZZER_PIN, 1000, 100);
  
  Serial.println("Setup complete. Robot ready to move!");
  
  // Play Caribbean Pirates melody to indicate successful setup
  playCaribbeanPiratesMelody();
  
  delay(2000);
}

//======================================================================
// MAIN LOOP - Contains the robot's movement sequence
//======================================================================
void loop() {
  // Update ADXL345 position tracking
  updateADXL345Position();
  
  // Reset position tracking before starting movement
  if (adxl345_found) {
    resetADXL345Position();
    Serial.println("ADXL345 position tracking reset for new movement sequence");
  }
  
  // Execute the user-defined movement plan
  Serial.println("=== EXECUTING MOVEMENT PLAN ===");
  Serial.print("Total movements in plan: ");
  Serial.println(MOVEMENT_PLAN_SIZE);
  
  for (int i = 0; i < MOVEMENT_PLAN_SIZE; i++) {
    Serial.print("Step ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(movementPlan[i]);
    
    // Execute the current movement command
    if (!executeMovementCommand(movementPlan[i])) {
      Serial.println("Movement failed! Stopping sequence.");
      stopMotors();
      return;
    }
    
    // Small delay between movements
    delay(500);
  }
  
  Serial.println("=== MOVEMENT PLAN COMPLETE ===");
  stopMotors();
  
  // Play mission completion melody
  playMissionCompleteMelody();
  
  // Final ADXL345 position report
  if (adxl345_found) {
    Serial.println("\n=== FINAL ADXL345 POSITION REPORT ===");
    Serial.print("Final Position: ");
    Serial.println(getADXL345PositionString());
    Serial.print("Total Distance Traveled: ");
    Serial.print(getADXL345Distance());
    Serial.println(" meters");
    Serial.println("=====================================");
  }
}

//======================================================================
// AUDIO FEEDBACK FUNCTIONS
//======================================================================

/**
 * Plays a single short beep for obstacle detection.
 */
void playObstacleBeep() {
  Serial.println("! OBSTACLE BEEP SIGNAL !");
  tone(BUZZER_PIN, 800, 150); // 800Hz for 150ms (short beep)
  delay(150);
  noTone(BUZZER_PIN);
}

/**
 * Plays two long beeps for ground/cliff detection.
 */
void playGroundBeep() {
  Serial.println("! GROUND/CLIFF BEEP SIGNAL !");
  for (int i = 0; i < 2; i++) {
    tone(BUZZER_PIN, 600, 500); // 600Hz for 500ms (long beep)
    delay(500);
    noTone(BUZZER_PIN);
    delay(200); // Pause between beeps
  }
}

/**
 * Plays the Caribbean Pirates melody for mission completion.
 */
void playMissionCompleteMelody() {
  Serial.println("=== MISSION COMPLETE - PLAYING CELEBRATION MELODY ===");
  playCaribbeanPiratesMelody();
}

//======================================================================
// MOVEMENT & SENSOR FUNCTIONS
//======================================================================

/**
 * Executes a movement command from the movement plan.
 * @param command The movement command string (e.g., "FWD_200", "RT_90", etc.)
 * @return True if movement was successful, false if failed
 */
bool executeMovementCommand(String command) {
  Serial.print("Executing command: ");
  Serial.println(command);
  
  // Parse the command
  if (command.startsWith("FWD_")) {
    // Forward movement
    String distanceStr = command.substring(4); // Remove "FWD_"
    float distanceCm = distanceStr.toFloat();
    float distanceM = distanceCm / 100.0; // Convert cm to meters
    
    Serial.print("Moving forward ");
    Serial.print(distanceCm);
    Serial.println(" cm");
    
    moveForwardDistance(distanceM);
    return true;
    
  } else if (command.startsWith("BKWD_")) {
    // Backward movement
    String distanceStr = command.substring(5); // Remove "BKWD_"
    float distanceCm = distanceStr.toFloat();
    float distanceM = distanceCm / 100.0; // Convert cm to meters
    
    Serial.print("Moving backward ");
    Serial.print(distanceCm);
    Serial.println(" cm");
    
    moveBackwardDistance(distanceM);
    return true;
    
  } else if (command.startsWith("LT_")) {
    // Left turn
    String angleStr = command.substring(3); // Remove "LT_"
    float angle = angleStr.toFloat();
    
    Serial.print("Turning left ");
    Serial.print(angle);
    Serial.println(" degrees");
    
    turnLeftAngle(angle);
    return true;
    
  } else if (command.startsWith("RT_")) {
    // Right turn
    String angleStr = command.substring(3); // Remove "RT_"
    float angle = angleStr.toFloat();
    
    Serial.print("Turning right ");
    Serial.print(angle);
    Serial.println(" degrees");
    
    turnRightAngle(angle);
    return true;
    
  } else {
    // Unknown command
    Serial.print("Unknown command: ");
    Serial.println(command);
    return false;
  }
}

/**
 * Moves the robot forward for a specific distance using ADXL345 tracking.
 * @param targetDistanceMeters The distance to travel in meters.
 */
void moveForwardDistance(float targetDistanceMeters) {
  Serial.print("Moving forward ");
  Serial.print(targetDistanceMeters);
  Serial.println(" meters...");
  
  // Reset position tracking for this movement
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Start forward movement
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  
  unsigned long startTime = millis();
  float distanceTraveled = 0.0;
  
  while (distanceTraveled < targetDistanceMeters) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Get current distance traveled
    if (adxl345_found) {
      distanceTraveled = getADXL345Distance();
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      distanceTraveled = (elapsedTime / 1000.0) * 0.4; // Assume 0.4 m/s speed
    }
    
    // Check for forward obstacles
    int forwardDistance = getForwardDistance();
    Serial.print("Forward distance: ");
    Serial.print(forwardDistance);
    Serial.print(" cm, Traveled: ");
    Serial.print(distanceTraveled);
    Serial.println(" m");
    
    // Check for ground/cliff
    long groundDistance = readSonar(GND_TRIG_PIN, GND_ECHO_PIN);
    Serial.print("Ground distance: ");
    Serial.print(groundDistance);
    Serial.println(" cm");
    
    // Report ADXL345 position if available
    if (adxl345_found) {
      Serial.print("ADXL345: ");
      Serial.println(getADXL345PositionString());
    }
    
    // Check for obstacles or holes
    bool obstacleDetected = (forwardDistance < OBSTACLE_THRESHOLD_CM);
    bool holeDetected = (groundDistance > CLIFF_THRESHOLD_CM);
    
    if (obstacleDetected) {
      Serial.println("! FORWARD OBSTACLE DETECTED!");
      playObstacleBeep(); // Play obstacle beep signal
      stopMotors();
      if (avoidObstacle()) {
        // Resume movement after avoidance
        Serial.println("Resuming forward movement...");
        motor1.setSpeed(255);
        motor2.setSpeed(255);
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        // Reset position tracking after avoidance
        if (adxl345_found) {
          resetADXL345Position();
        }
        startTime = millis();
      } else {
        Serial.println("Could not avoid obstacle. Stopping movement.");
        return;
      }
    }
    
    if (holeDetected) {
      Serial.println("! HOLE/CLIFF DETECTED! No ground in front!");
      playGroundBeep(); // Play ground/cliff beep signal
      stopMotors();
      if (avoidObstacle()) {
        // Resume movement after avoidance
        Serial.println("Resuming forward movement...");
        motor1.setSpeed(255);
        motor2.setSpeed(255);
        motor1.run(FORWARD);
        motor2.run(FORWARD);
        // Reset position tracking after avoidance
        if (adxl345_found) {
          resetADXL345Position();
        }
        startTime = millis();
      } else {
        Serial.println("Could not avoid hole. Stopping movement.");
        return;
      }
    }
    
    delay(100); // Check every 100ms
  }
  
  // Stop motors after reaching target distance
  stopMotors();
  Serial.print("Forward movement complete. Distance traveled: ");
  Serial.print(distanceTraveled);
  Serial.println(" meters");
}

/**
 * Moves the robot backward for a specific distance using ADXL345 tracking.
 * @param targetDistanceMeters The distance to travel in meters.
 */
void moveBackwardDistance(float targetDistanceMeters) {
  Serial.print("Moving backward ");
  Serial.print(targetDistanceMeters);
  Serial.println(" meters...");
  
  // Reset position tracking for this movement
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Start backward movement
  motor1.run(BACKWARD);
  motor2.run(BACKWARD);
  
  unsigned long startTime = millis();
  float distanceTraveled = 0.0;
  
  while (distanceTraveled < targetDistanceMeters) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Get current distance traveled
    if (adxl345_found) {
      distanceTraveled = getADXL345Distance();
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      distanceTraveled = (elapsedTime / 1000.0) * 0.4; // Assume 0.4 m/s speed
    }
    
    // Check for obstacles behind (using forward sonar as approximation)
    int forwardDistance = getForwardDistance();
    Serial.print("Forward distance: ");
    Serial.print(forwardDistance);
    Serial.print(" cm, Traveled: ");
    Serial.print(distanceTraveled);
    Serial.println(" m");
    
    // Report ADXL345 position if available
    if (adxl345_found) {
      Serial.print("ADXL345: ");
      Serial.println(getADXL345PositionString());
    }
    
    delay(100); // Check every 100ms
  }
  
  // Stop motors after reaching target distance
  stopMotors();
  Serial.print("Backward movement complete. Distance traveled: ");
  Serial.print(distanceTraveled);
  Serial.println(" meters");
}

/**
 * Turns the robot right using time-based navigation.
 */
void turnRight() {
  Serial.println("Turning right...");
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Right turn: motor1 forward, motor2 backward
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  
  unsigned long startTime = millis();
  unsigned long turnTimeMillis = 2000; // 2 seconds like in motor_move.ino
  
  while (millis() - startTime < turnTimeMillis) {
    delay(50);
  }
  
  // Stop motors after turn
  stopMotors();
  Serial.println("Right turn complete.");
}

/**
 * Attempts to avoid obstacles by checking both forward and ground sonars.
 * Chooses the side with ground to go around obstacles.
 * @return True if avoidance was successful, false otherwise.
 */
bool avoidObstacle() {
  Serial.println("=== OBSTACLE AVOIDANCE STARTED ===");
  
  // Check forward obstacle
  long forwardDistance = readSonar(FWD_TRIG_PIN, FWD_ECHO_PIN);
  Serial.print("Forward obstacle distance: ");
  Serial.print(forwardDistance);
  Serial.println(" cm");
  
  // Check ground in front
  long groundDistance = readSonar(GND_TRIG_PIN, GND_ECHO_PIN);
  Serial.print("Ground distance: ");
  Serial.print(groundDistance);
  Serial.println(" cm");
  
  // Determine if there's a hole (no ground)
  bool holeDetected = (groundDistance > CLIFF_THRESHOLD_CM);
  
  if (holeDetected) {
    Serial.println("! HOLE DETECTED! No ground in front!");
  }
  
  // Check left side for ground
  Serial.println("Checking left side for ground...");
  long leftGroundDistance = checkLeftSideGround();
  Serial.print("Left side ground distance: ");
  Serial.print(leftGroundDistance);
  Serial.println(" cm");
  
  // Check right side for ground
  Serial.println("Checking right side for ground...");
  long rightGroundDistance = checkRightSideGround();
  Serial.print("Right side ground distance: ");
  Serial.print(rightGroundDistance);
  Serial.println(" cm");
  
  // Determine which side has ground
  bool leftHasGround = (leftGroundDistance <= CLIFF_THRESHOLD_CM);
  bool rightHasGround = (rightGroundDistance <= CLIFF_THRESHOLD_CM);
  
  Serial.print("Left side has ground: ");
  Serial.println(leftHasGround ? "YES" : "NO");
  Serial.print("Right side has ground: ");
  Serial.println(rightHasGround ? "YES" : "NO");
  
  // Choose direction based on ground availability
  if (leftHasGround && rightHasGround) {
    // Both sides have ground, choose the side with more space
    if (leftGroundDistance > rightGroundDistance) {
      Serial.println("Both sides have ground. Choosing LEFT (more space)");
      turnLeftAngle(45); // Turn 45 degrees to avoid
      return true;
    } else {
      Serial.println("Both sides have ground. Choosing RIGHT (more space)");
      turnRightAngle(45); // Turn 45 degrees to avoid
      return true;
    }
  } else if (leftHasGround) {
    // Only left side has ground
    Serial.println("Only LEFT side has ground. Turning left to avoid obstacle/hole");
    turnLeftAngle(45); // Turn 45 degrees to avoid
    return true;
  } else if (rightHasGround) {
    // Only right side has ground
    Serial.println("Only RIGHT side has ground. Turning right to avoid obstacle/hole");
    turnRightAngle(45); // Turn 45 degrees to avoid
    return true;
  } else {
    // No ground on either side - dangerous situation
    Serial.println("! DANGER! No ground on either side!");
    playGroundBeep(); // Play ground/cliff beep signal for dangerous situation
    Serial.println("Stopping robot for safety");
    stopMotors();
    return false;
  }
}

/**
 * Turns the robot left using time-based navigation.
 */
void turnLeft() {
  Serial.println("Turning left...");
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Left turn: motor1 backward, motor2 forward
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  
  unsigned long startTime = millis();
  unsigned long turnTimeMillis = 2000; // 2 seconds like in motor_move.ino
  
  while (millis() - startTime < turnTimeMillis) {
    delay(50);
  }
  
  // Stop motors after turn
  stopMotors();
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
 * Checks the left side for ground using the ground sonar.
 * Robot turns left slightly to check ground on left side.
 * @return Ground distance in centimeters.
 */
long checkLeftSideGround() {
  // Turn left slightly to check ground on left side
  Serial.println("Turning left to check ground on left side...");
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Left turn: motor1 backward, motor2 forward
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  
  // Turn for a short time to check left side (45 degrees)
  delay(1000); // 1 second for ~45 degree turn
  
  // Stop motors
  stopMotors();
  
  // Read ground distance
  long groundDistance = readSonar(GND_TRIG_PIN, GND_ECHO_PIN);
  
  // Turn back to original direction
  Serial.println("Turning back to original direction...");
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  delay(1000); // 1 second back
  
  stopMotors();
  
  return groundDistance;
}

/**
 * Checks the right side for ground using the ground sonar.
 * Robot turns right slightly to check ground on right side.
 * @return Ground distance in centimeters.
 */
long checkRightSideGround() {
  // Turn right slightly to check ground on right side
  Serial.println("Turning right to check ground on right side...");
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Right turn: motor1 forward, motor2 backward
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  
  // Turn for a short time to check right side (45 degrees)
  delay(1000); // 1 second for ~45 degree turn
  
  // Stop motors
  stopMotors();
  
  // Read ground distance
  long groundDistance = readSonar(GND_TRIG_PIN, GND_ECHO_PIN);
  
  // Turn back to original direction
  Serial.println("Turning back to original direction...");
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  delay(1000); // 1 second back
  
  stopMotors();
  
  return groundDistance;
}

/**
 * Gets the forward distance from the forward sonar sensor.
 * @return Distance in centimeters.
 */
int getForwardDistance() {
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
// UTILITY & HELPER FUNCTIONS
//======================================================================

/**
 * Stops both motors.
 */
void stopMotors() {
  motor1.setSpeed(0);
  motor1.run(RELEASE);
  motor2.setSpeed(0);
  motor2.run(RELEASE);
}

/**
 * Provides information about motor configuration.
 */
void printMotorShieldInfo() {
  Serial.println("\n=== MOTOR CONFIGURATION ===");
  Serial.println("Using AFMotor library with DC motors.");
  Serial.println("Motor Configuration:");
  Serial.println("  Left Motor:  AF_DCMotor(1)");
  Serial.println("  Right Motor: AF_DCMotor(2)");
  Serial.println("=====================================\n");
}

//======================================================================
// ADXL345 ACCELEROMETER FUNCTIONS
//======================================================================

/**
 * Initializes the ADXL345 accelerometer.
 * @return True if ADXL345 is found and initialized, false otherwise.
 */
bool initializeADXL345() {
  // Check if device responds
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x00); // Device ID register
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, 1);
  
  if (Wire.available()) {
    byte deviceId = Wire.read();
    if (deviceId == ADXL345_DEVID) {
      Serial.print("ADXL345 Device ID: 0x");
      Serial.println(deviceId, HEX);
      
      // Set measurement mode
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x2D); // Power control register
      Wire.write(0x08); // Set measure bit
      Wire.endTransmission();
      
      // Set data format (16g range, full resolution)
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x31); // Data format register
      Wire.write(0x0B); // 16g range, full resolution
      Wire.endTransmission();
      
      // Set data rate (100Hz)
      Wire.beginTransmission(ADXL345_ADDR);
      Wire.write(0x2C); // Data rate register
      Wire.write(0x0A); // 100Hz
      Wire.endTransmission();
      
      delay(100); // Wait for settings to take effect
      return true;
    } else {
      Serial.print("Wrong Device ID: 0x");
      Serial.println(deviceId, HEX);
      return false;
    }
  } else {
    Serial.println("No response from ADXL345");
    return false;
  }
}

/**
 * Calibrates the ADXL345 accelerometer by taking multiple samples.
 * @param samples Number of samples to take for calibration.
 */
void calibrateADXL345(int samples) {
  float sum[3] = {0, 0, 0};
  
  Serial.print("Taking ");
  Serial.print(samples);
  Serial.println(" calibration samples...");
  
  for (int i = 0; i < samples; i++) {
    float accel[3];
    readADXL345Raw(accel);
    
    sum[0] += accel[0];
    sum[1] += accel[1];
    sum[2] += accel[2];
    
    if (i % 50 == 0) {
      Serial.print("Calibration progress: ");
      Serial.print(i);
      Serial.print("/");
      Serial.println(samples);
    }
    
    delay(10); // Small delay between readings
  }
  
  // Calculate average offsets
  adxl345_offset[0] = sum[0] / samples;
  adxl345_offset[1] = sum[1] / samples;
  adxl345_offset[2] = sum[2] / samples;
  
  Serial.println("Calibration offsets calculated:");
  Serial.print("X offset: ");
  Serial.println(adxl345_offset[0]);
  Serial.print("Y offset: ");
  Serial.println(adxl345_offset[1]);
  Serial.print("Z offset: ");
  Serial.println(adxl345_offset[2]);
}

/**
 * Reads raw accelerometer data from ADXL345.
 * @param accel Array to store X, Y, Z acceleration values.
 */
void readADXL345Raw(float accel[3]) {
  Wire.beginTransmission(ADXL345_ADDR);
  Wire.write(0x32); // Start from DATAX0 register
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345_ADDR, 6);
  
  if (Wire.available() >= 6) {
    int16_t rawX = Wire.read() | (Wire.read() << 8);
    int16_t rawY = Wire.read() | (Wire.read() << 8);
    int16_t rawZ = Wire.read() | (Wire.read() << 8);
    
    // Convert to g (16g range, 13-bit resolution)
    accel[0] = rawX * 0.0039; // 16g / 4096
    accel[1] = rawY * 0.0039;
    accel[2] = rawZ * 0.0039;
  }
}

/**
 * Reads calibrated accelerometer data.
 * @param accel Array to store calibrated X, Y, Z acceleration values.
 */
void readADXL345Calibrated(float accel[3]) {
  readADXL345Raw(accel);
  
  // Apply calibration offsets
  accel[0] -= adxl345_offset[0];
  accel[1] -= adxl345_offset[1];
  accel[2] -= adxl345_offset[2];
}

/**
 * Updates position and velocity using accelerometer data.
 * Should be called regularly in the main loop.
 */
void updateADXL345Position() {
  if (!adxl345_found) return;
  
  unsigned long currentTime = millis();
  if (currentTime - last_adxl345_read >= ADXL345_READ_INTERVAL) {
    float accel[3];
    readADXL345Calibrated(accel);
    
    // Calculate time delta
    float dt = (currentTime - last_adxl345_read) / 1000.0; // Convert to seconds
    
    // Update velocity (integrate acceleration)
    adxl345_velocity[0] += accel[0] * 9.81 * dt; // Convert g to m/s²
    adxl345_velocity[1] += accel[1] * 9.81 * dt;
    adxl345_velocity[2] += accel[2] * 9.81 * dt;
    
    // Update position (integrate velocity)
    adxl345_position[0] += adxl345_velocity[0] * dt;
    adxl345_position[1] += adxl345_velocity[1] * dt;
    adxl345_position[2] += adxl345_velocity[2] * dt;
    
    last_adxl345_read = currentTime;
  }
}

/**
 * Gets the current distance traveled (magnitude of position).
 * @return Distance in meters.
 */
float getADXL345Distance() {
  if (!adxl345_found) return 0.0;
  
  float distance = sqrt(
    adxl345_position[0] * adxl345_position[0] +
    adxl345_position[1] * adxl345_position[1] +
    adxl345_position[2] * adxl345_position[2]
  );
  
  return distance;
}

/**
 * Gets the current position as a string for debugging.
 * @return String with X, Y, Z position values.
 */
String getADXL345PositionString() {
  if (!adxl345_found) return "ADXL345 not available";
  
  String pos = "Pos(X:";
  pos += adxl345_position[0];
  pos += "m, Y:";
  pos += adxl345_position[1];
  pos += "m, Z:";
  pos += adxl345_position[2];
  pos += "m)";
  
  return pos;
}

/**
 * Resets the position and velocity tracking.
 */
void resetADXL345Position() {
  if (!adxl345_found) return;
  
  adxl345_position[0] = 0;
  adxl345_position[1] = 0;
  adxl345_position[2] = 0;
  adxl345_velocity[0] = 0;
  adxl345_velocity[1] = 0;
  adxl345_velocity[2] = 0;
  
  Serial.println("ADXL345 position tracking reset");
}

/**
 * Turns the robot right by a specific angle using ADXL345 for angle tracking.
 * @param targetAngle The angle to turn in degrees.
 */
void turnRightAngle(float targetAngle) {
  Serial.print("Turning right ");
  Serial.print(targetAngle);
  Serial.println(" degrees...");
  
  // Reset position tracking for this turn
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Right turn: motor1 forward, motor2 backward
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  
  unsigned long startTime = millis();
  float angleTurned = 0.0;
  
  while (angleTurned < targetAngle) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Calculate angle turned based on position change
    if (adxl345_found) {
      // Use Y position change to estimate angle (assuming robot turns in X-Y plane)
      float yChange = abs(adxl345_position[1]);
      // Rough estimation: 1 meter Y change ≈ 90 degrees for typical robot dimensions
      angleTurned = (yChange / 1.0) * 90.0;
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      angleTurned = (elapsedTime / 1000.0) * 45.0; // Assume 45 degrees per second
    }
    
    Serial.print("Angle turned: ");
    Serial.print(angleTurned);
    Serial.println(" degrees");
    
    delay(50); // Check every 50ms for smoother turning
  }
  
  // Stop motors after reaching target angle
  stopMotors();
  Serial.print("Right turn complete. Final angle: ");
  Serial.print(angleTurned);
  Serial.println(" degrees");
}

/**
 * Turns the robot left by a specific angle using ADXL345 for angle tracking.
 * @param targetAngle The angle to turn in degrees.
 */
void turnLeftAngle(float targetAngle) {
  Serial.print("Turning left ");
  Serial.print(targetAngle);
  Serial.println(" degrees...");
  
  // Reset position tracking for this turn
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Left turn: motor1 backward, motor2 forward
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  
  unsigned long startTime = millis();
  float angleTurned = 0.0;
  
  while (angleTurned < targetAngle) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Calculate angle turned based on position change
    if (adxl345_found) {
      // Use Y position change to estimate angle (assuming robot turns in X-Y plane)
      float yChange = abs(adxl345_position[1]);
      // Rough estimation: 1 meter Y change ≈ 90 degrees for typical robot dimensions
      angleTurned = (yChange / 1.0) * 90.0;
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      angleTurned = (elapsedTime / 1000.0) * 45.0; // Assume 45 degrees per second
    }
    
    Serial.print("Angle turned: ");
    Serial.print(angleTurned);
    Serial.println(" degrees");
    
    delay(50); // Check every 50ms for smoother turning
  }
  
  // Stop motors after reaching target angle
  stopMotors();
  Serial.print("Left turn complete. Final angle: ");
  Serial.print(angleTurned);
  Serial.println(" degrees");
}

/**
 * Turns the robot right exactly 90 degrees using ADXL345 for angle tracking.
 */
void turnRight90() {
  Serial.println("Turning right 90 degrees...");
  
  // Reset position tracking for this turn
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Right turn: motor1 forward, motor2 backward
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  
  unsigned long startTime = millis();
  float angleTurned = 0.0;
  const float targetAngle = 90.0; // 90 degrees
  
  while (angleTurned < targetAngle) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Calculate angle turned based on position change
    if (adxl345_found) {
      // Use Y position change to estimate angle (assuming robot turns in X-Y plane)
      float yChange = abs(adxl345_position[1]);
      // Rough estimation: 1 meter Y change ≈ 90 degrees for typical robot dimensions
      angleTurned = (yChange / 1.0) * 90.0;
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      angleTurned = (elapsedTime / 1000.0) * 45.0; // Assume 45 degrees per second
    }
    
    Serial.print("Angle turned: ");
    Serial.print(angleTurned);
    Serial.println(" degrees");
    
    delay(50); // Check every 50ms for smoother turning
  }
  
  // Stop motors after reaching target angle
  stopMotors();
  Serial.print("Right turn complete. Final angle: ");
  Serial.print(angleTurned);
  Serial.println(" degrees");
}

/**
 * Turns the robot left exactly 90 degrees using ADXL345 for angle tracking.
 */
void turnLeft90() {
  Serial.println("Turning left 90 degrees...");
  
  // Reset position tracking for this turn
  if (adxl345_found) {
    resetADXL345Position();
  }
  
  // Set motor speed to 255 (100%) like in motor_move.ino
  motor1.setSpeed(255);
  motor2.setSpeed(255);
  
  // Left turn: motor1 backward, motor2 forward
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  
  unsigned long startTime = millis();
  float angleTurned = 0.0;
  const float targetAngle = 90.0; // 90 degrees
  
  while (angleTurned < targetAngle) {
    // Update ADXL345 position tracking
    updateADXL345Position();
    
    // Calculate angle turned based on position change
    if (adxl345_found) {
      // Use Y position change to estimate angle (assuming robot turns in X-Y plane)
      float yChange = abs(adxl345_position[1]);
      // Rough estimation: 1 meter Y change ≈ 90 degrees for typical robot dimensions
      angleTurned = (yChange / 1.0) * 90.0;
    } else {
      // Fallback to time-based if ADXL345 not available
      unsigned long elapsedTime = millis() - startTime;
      angleTurned = (elapsedTime / 1000.0) * 45.0; // Assume 45 degrees per second
    }
    
    Serial.print("Angle turned: ");
    Serial.print(angleTurned);
    Serial.println(" degrees");
    
    delay(50); // Check every 50ms for smoother turning
  }
  
  // Stop motors after reaching target angle
  stopMotors();
  Serial.print("Left turn complete. Final angle: ");
  Serial.print(angleTurned);
  Serial.println(" degrees");
}

//======================================================================
// CARIBBEAN PIRATES MELODY FUNCTIONS
//======================================================================

// Musical note definitions
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0

/**
 * Plays the Caribbean Pirates melody on the buzzer.
 * This function plays once at the end of successful setup.
 */
void playCaribbeanPiratesMelody() {
  Serial.println("Playing Caribbean Pirates melody...");
  
  // Pirates of the Caribbean - "He's a Pirate" Theme
  // Notes and durations
  int melody[] = {
    NOTE_D4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,
    
    NOTE_D4, NOTE_G4, NOTE_A4, NOTE_A4, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0,
    NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
    NOTE_A4, NOTE_G4, NOTE_A4, 0,
    
    NOTE_E5, NOTE_D5, NOTE_A4, 0,
    NOTE_G4, NOTE_E4, NOTE_D4, 0,
    NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5, 0,
    NOTE_C5, NOTE_C5, NOTE_C5, NOTE_E5, NOTE_D5, 0,
    
    NOTE_A4, NOTE_A4, NOTE_A4, NOTE_B4, NOTE_C5, 0,
    NOTE_C5, NOTE_C5, NOTE_C5, NOTE_E5, NOTE_D5, 0,
    NOTE_A4, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_E5, 0,
    NOTE_G5, NOTE_F5, NOTE_E5, NOTE_D5, NOTE_C5, 0
  };
  
  int noteDurations[] = {
    8, 8, 4, 8, 8,
    8, 8, 4, 8, 8,
    8, 8, 4, 8, 8,
    4, 8, 4, 8,
    
    8, 8, 4, 8, 8,
    8, 8, 4, 8, 8,
    8, 8, 4, 8, 8,
    4, 8, 4, 8,
    
    4, 8, 4, 8,
    4, 8, 4, 8,
    8, 8, 8, 8, 4, 8,
    8, 8, 8, 8, 4, 8,
    
    8, 8, 8, 8, 4, 8,
    8, 8, 8, 8, 4, 8,
    8, 8, 8, 8, 4, 8,
    8, 8, 8, 8, 4, 8
  };
  
  // Play the melody
  for (int i = 0; i < sizeof(melody) / sizeof(melody[0]); i++) {
    int noteDuration = 1000 / noteDurations[i];
    
    if (melody[i] != 0) {
      tone(BUZZER_PIN, melody[i], noteDuration);
    }
    
    // Wait for the note to finish plus a small pause
    delay(noteDuration * 1.3);
    noTone(BUZZER_PIN);
  }
  
  Serial.println("Caribbean Pirates melody complete!");
}
