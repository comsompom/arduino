#include <Servo.h>
Servo myservo;

void setup() {
  myservo.attach(9); // Attach the servo to pin 9
}

void loop() {
  for (int i = 0; i <= 180; i++) { // Rotate the servo from 0 to 180 degrees
    myservo.write(i);
    delay(15);
  }
  for (int i = 180; i >= 0; i--) { // Rotate the servo back to 0 degrees
    myservo.write(i);
    delay(15);
  }
}