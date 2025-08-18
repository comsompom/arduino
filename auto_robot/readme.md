
### **Important Preliminary Notes & Assumptions**

1.  **Gyro "GY-65":** The "GY-65" is typically a **BMP085/BMP180 barometric pressure sensor**, which measures altitude, not rotation or distance. This is likely a typo in your request. A very common and suitable sensor for this task is the **MPU-6050** (often on a GY-521 board), which contains both a gyroscope and an accelerometer. **This solution is written for an MPU-6050.** If you truly have a different sensor, the library and code for reading the gyroscope will need to be changed.
2.  **Gyro for Distance:** A gyroscope **cannot** measure linear distance (meters/cm). It measures *angular velocity* (how fast it's turning). We use it to make accurate turns (e.g., exactly 90 degrees). To measure distance, we will use a time-based approach. This requires **calibration**. You will need to measure how far your robot travels in one second and update a constant in the code. For higher accuracy, motor encoders are recommended, but they were not in your equipment list.
3.  **Motor Shield:** This solution assumes you are using the popular **Adafruit Motor Shield V2**. If you are using a different shield (like an L298N-based one), the motor control functions will need to be completely rewritten.
4.  **Sensor Shield:** The "DK Elektroniks" sensor shield simply makes it easier to connect wires. I will specify the direct Arduino Mega pins to use. You can then find the corresponding pins on your shield.

---

### **Step 1: Required Libraries**

You need to install the following libraries in your Arduino IDE. Go to `Sketch > Include Library > Manage Libraries...` and search for and install each of these:

1.  `Adafruit Motor Shield V2 Library` by Adafruit
2.  `Adafruit Unified Sensor` by Adafruit
3.  `I2Cdev` by Jeff Rowberg
4.  `MPU6050` by Jeff Rowberg (This might be part of the I2Cdev library installation)

---

### **Step 2: Hardware Connections**

| Component            | Connection to Arduino Mega 2560 / Shields                                                                      |
| -------------------- | -------------------------------------------------------------------------------------------------------------- |
| **Motor Shield**     | Plug it directly on top of the Arduino Mega.                                                                   |
| **Power Supply**     | Connect your motor power supply (e.g., 7.4V LiPo battery) to the Motor Shield's power terminal.                  |
| **Left Motor**       | Connect to Motor Shield terminals `M1` or `M2`.                                                                |
| **Right Motor**      | Connect to Motor Shield terminals `M3` or `M4`.                                                                |
| **MPU-6050 (Gyro)**  | This is an I2C device. Connect it to the Mega's I2C pins: `SDA` -> `SDA (Pin 20)`, `SCL` -> `SCL (Pin 21)`.        |
| **Buzzer**           | `Positive (+)` -> `Pin 22`, `Negative (-)` -> `GND`.                                                             |
| **Forward Sonar**    | `VCC` -> `5V`, `GND` -> `GND`, `Trig` -> `Pin 30`, `Echo` -> `Pin 31`.                                             |
| **Ground Sonar**     | `VCC` -> `5V`, `GND` -> `GND`, `Trig` -> `Pin 32`, `Echo` -> `Pin 33`.                                             |

**Note:** Use your Sensor Shield to make these connections to the specified pins neatly. Ensure all components share a common `GND`.

---

### **Step 3: The Arduino Sketch**

Copy and paste the following code into your Arduino IDE. Read the comments carefully, especially the section on **CALIBRATION**.

```cpp
//======================================================================
// LIBRARIES
//======================================================================
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "I2Cdev.h"
#include "MPU6050.h"

//======================================================================
// PIN & OBJECT DEFINITIONS
//======================================================================
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which shield connections to use for the motors
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4); // M4

// MPU6050 Gyroscope object
MPU6050 accelgyro;

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

// Gyro calibration
const int GYRO_CALIBRATION_SAMPLES = 200;
int16_t gyro_z_offset = 0;

// Gyro turning variables
float yawAngle = 0;
unsigned long prevTime = 0;

//======================================================================
// SETUP FUNCTION - Runs once at the beginning
//======================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Robot starting up...");

  // --- Initial Setup & Beep ---
  pinMode(BUZZER_PIN, OUTPUT);
  beep(1, 250); // Beep once on power-up

  // --- Motor Shield Setup ---
  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Halting.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // --- Sonar Pin Setup ---
  pinMode(FWD_TRIG_PIN, OUTPUT);
  pinMode(FWD_ECHO_PIN, INPUT);
  pinMode(GND_TRIG_PIN, OUTPUT);
  pinMode(GND_ECHO_PIN, INPUT);

  // --- Gyroscope Setup & Calibration ---
  Wire.begin();
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  
  calibrateGyro();

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
  turnRightWithGyro(90);

  Serial.println("Step 3: Moving forward 50 cm...");
  moveForwardWithObstacleCheck(50);

  Serial.println("Step 4: Turning right 90 degrees...");
  turnRightWithGyro(90);
  
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
      Serial.println("Obstacle detected! Halting this movement step.");
      stopMotors();
      // Here you can decide what to do. For now, we just stop this part of the sequence.
      // You could call an avoidance routine here instead.
      // avoidObstacle();
      return; // Exit the function
    }
    
    // If no obstacle, keep moving forward
    setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
    delay(20); // Small delay to prevent overwhelming the loop
  }
  
  // Stop after the time has elapsed
  stopMotors();
}

/**
 * Turns the robot right by a specified angle using the gyroscope.
 * @param targetAngle The angle in degrees to turn.
 */
void turnRightWithGyro(float targetAngle) {
  yawAngle = 0; // Reset angle
  prevTime = micros();
  
  // Start turning: right motor backward, left motor forward
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);

  while (abs(yawAngle) < targetAngle * 0.95) { // Stop slightly short to account for momentum
    unsigned long currentTime = micros();
    float dt = (currentTime - prevTime) / 1000000.0; // Delta time in seconds
    prevTime = currentTime;

    int16_t gx, gy, gz, ax, ay, az;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    // Use the calibrated Z-axis gyro data
    float gyroZ = (gz - gyro_z_offset) / 131.0; // Convert to degrees per second

    // Integrate to get angle
    yawAngle += gyroZ * dt;

    // Optional: Print angle for debugging
    // Serial.print("Current Angle: ");
    // Serial.println(yawAngle);
  }
  stopMotors();
  delay(200); // Pause after turn
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
 * Calibrates the gyroscope by taking multiple readings and finding the average offset.
 */
void calibrateGyro() {
  Serial.println("Calibrating Gyroscope. Keep the robot stationary...");
  long z_total = 0;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    int16_t gx, gy, gz, ax, ay, az;
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    z_total += gz;
    delay(5);
  }
  gyro_z_offset = z_total / GYRO_CALIBRATION_SAMPLES;
  Serial.print("Gyro Z-axis offset calculated: ");
  Serial.println(gyro_z_offset);
}

/**
 * Controls both motors at a given speed.
 * @param leftSpeed Speed for the left motor (-255 to 255).
 * @param rightSpeed Speed for the right motor (-255 to 255).
 */
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Control Left Motor
  if (leftSpeed >= 0) {
    leftMotor->setSpeed(leftSpeed);
    leftMotor->run(FORWARD);
  } else {
    leftMotor->setSpeed(-leftSpeed);
    leftMotor->run(BACKWARD);
  }

  // Control Right Motor
  if (rightSpeed >= 0) {
    rightMotor->setSpeed(rightSpeed);
    rightMotor->run(FORWARD);
  } else {
    rightMotor->setSpeed(-rightSpeed);
    rightMotor->run(BACKWARD);
  }
}

/**
 * Stops both motors.
 */
void stopMotors() {
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

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
```

---

### **Step 4: How to Calibrate and Use**

This is the most critical step to make your robot work correctly.

1.  **Upload the Code:** Upload the sketch to your Arduino Mega 2560.
2.  **Power On:** Power up the robot. You should hear:
    *   One single beep immediately.
    *   A pause of a few seconds while the gyroscope calibrates (keep the robot perfectly still!). You will see messages in the Serial Monitor.
    *   Two short beeps to signal that calibration is done and it's ready to run.
3.  **CALIBRATE THE SPEED (MANDATORY):**
    *   Find this line in the code: `const float ROBOT_SPEED_CM_PER_S = 25.0;`
    *   Temporarily comment out the entire `loop()` function's content and replace it with this:
        ```cpp
        void loop() {
          setMotorSpeed(150, 150); // Use the same speed as MOTOR_SPEED constant
          delay(2000); // Run for 2 seconds
          stopMotors();
          while(1); // Stop
        }
        ```
    *   Upload this modified code. When you power it on (after the beeps), the robot will move forward for exactly 2 seconds and then stop.
    *   **Measure the distance it traveled** in centimeters with a ruler.
    *   **Calculate your speed:** `Your_Speed = (Distance_Traveled_in_cm) / 2.0;`
    *   Update the `ROBOT_SPEED_CM_PER_S` constant with your calculated value.
    *   **Restore the original `loop()` function** and upload the code again. Now your robot's distance calculations will be accurate.
4.  **Tweak Turning (Optional):**
    *   If your robot over-turns or under-turns, adjust this line inside the `turnRightWithGyro()` function:
        `while (abs(yawAngle) < targetAngle * 0.95)`
    *   If it **under-turns** (doesn't turn enough), increase `0.95` slightly (e.g., to `0.97`).
    *   If it **over-turns**, decrease `0.95` (e.g., to `0.92`). This compensates for motor momentum.
5.  **Adjust Obstacle Thresholds:**
    *   Change `OBSTACLE_THRESHOLD_CM` and `CLIFF_THRESHOLD_CM` at the top of the code to suit your environment.

