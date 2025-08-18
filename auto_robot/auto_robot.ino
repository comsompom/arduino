//======================================================================
// LIBRARIES
/Users/olegbourdo/Documents/Arduino/auto_robot/caribean_pirates_melody.ino: In function 'void setup()':
/Users/olegbourdo/Documents/Arduino/auto_robot/caribean_pirates_melody.ino:153:6: error: redefinition of 'void setup()'
 void setup() {
      ^~~~~
/Users/olegbourdo/Documents/Arduino/auto_robot/auto_robot.ino:61:6: note: 'void setup()' previously defined here
 void setup() {
      ^~~~~
/Users/olegbourdo/Documents/Arduino/auto_robot/caribean_pirates_melody.ino: In function 'void loop()':
/Users/olegbourdo/Documents/Arduino/auto_robot/caribean_pirates_melody.ino:158:6: error: redefinition of 'void loop()'
 void loop() {
      ^~~~
/Users/olegbourdo/Documents/Arduino/auto_robot/auto_robot.ino:122:6: note: 'void loop()' previously defined here
 void loop() {
      ^~~~
exit status 1

Compilation error: redefinition of 'void setup()'
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
  
  // Initialize sonar sensors
  pinMode(FWD_TRIG_PIN, OUTPUT);
  pinMode(FWD_ECHO_PIN, INPUT);
  pinMode(GND_TRIG_PIN, OUTPUT);
  pinMode(GND_ECHO_PIN, INPUT);
  Serial.println("Sonar sensors initialized.");
  
  // Initialize stepper motors
  Serial.println("Initializing stepper motors...");
  leftStepper.setSpeed(50);
  rightStepper.setSpeed(50);
  Serial.println("Stepper motors initialized.");
  
  // Test stepper motors briefly
  Serial.println("Testing stepper motors...");
  for (int i = 0; i < 10; i++) {
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    delay(10);
  }
  leftStepper.setSpeed(0);
  rightStepper.setSpeed(0);
  Serial.println("Stepper motor test complete.");
  
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
  delay(2000);
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
 * Moves the robot forward for a given distance using time-based navigation.
 * Continuously monitors for obstacles during movement.
 * @param distanceCm The distance to travel in centimeters.
 */
void moveForwardWithObstacleCheck(float distanceCm) {
  Serial.print("Moving forward "); Serial.print(distanceCm); Serial.println(" cm");
  
  // Calculate the time required to travel the distance based on the calibrated speed
  float durationSeconds = distanceCm / ROBOT_SPEED_CM_PER_S;
  long durationMillis = durationSeconds * 1000;

  unsigned long startTime = millis();
  unsigned long lastObstacleCheck = millis();
  
  // Set motors to move forward
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
  
  while (millis() - startTime < durationMillis) {
    // Check for obstacles every 100ms
    if (millis() - lastObstacleCheck > 100) {
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
            setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
            continue;
          }
        } else {
          Serial.println("Could not avoid obstacle. Stopping movement.");
          return; // Exit the function
        }
      }
      lastObstacleCheck = millis();
    }
    
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
    delay(50); // Small delay for stability
  }
  
  // Stop after the time has elapsed
  stopMotors();
  Serial.println("Forward movement complete.");
}

/**
 * Turns the robot right by a specified angle using time-based turning.
 * @param targetAngle The angle in degrees to turn.
 */
void turnRight(float targetAngle) {
  Serial.print("Turning right "); Serial.print(targetAngle); Serial.println(" degrees");
  
  // Calculate turn time based on angle (approximate)
  // Assuming 90 degrees takes about 2 seconds at TURN_SPEED
  float turnTimeSeconds = (targetAngle / 90.0) * 2.0;
  long turnTimeMillis = turnTimeSeconds * 1000;
  
  // Start turning: right motor backward, left motor forward
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < turnTimeMillis) {
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
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
 * Turns the robot left by a specified angle using time-based turning.
 * @param targetAngle The angle in degrees to turn.
 */
void turnLeft(float targetAngle) {
  Serial.print("Turning left "); Serial.print(targetAngle); Serial.println(" degrees");
  
  // Calculate turn time based on angle (approximate)
  // Assuming 90 degrees takes about 2 seconds at TURN_SPEED
  float turnTimeSeconds = (targetAngle / 90.0) * 2.0;
  long turnTimeMillis = turnTimeSeconds * 1000;
  
  // Start turning: left motor backward, right motor forward
  setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < turnTimeMillis) {
    // Run stepper motors
    leftStepper.runSpeed();
    rightStepper.runSpeed();
    
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
// UTILITY & HELPER FUNCTIONS
//======================================================================

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
