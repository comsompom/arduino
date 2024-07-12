// example of using the 4 quadrocopter motors

#include <AFMotor.h>

AF_DCMotor motor4(4); // white at the border black to center
AF_DCMotor motor3(3); // white at the border black to center
AF_DCMotor motor2(2); // red to center blue to border
AF_DCMotor motor1(1); // red to border blue to center

int motor_speed;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop() {
  uint8_t i;
  motor4.run(BACKWARD);
  motor3.run(BACKWARD);
  motor2.run(FORWARD);
  motor1.run(BACKWARD);

  motor_speed = 220;

  motor4.setSpeed(motor_speed);
  motor3.setSpeed(motor_speed);
  motor2.setSpeed(motor_speed);
  motor1.setSpeed(motor_speed);

  for (i=0; i<255; i++) {
    delay(1);
  }
  
  motor4.run(RELEASE);
  motor3.run(RELEASE);
  motor2.run(RELEASE);
  motor1.run(RELEASE);

}
