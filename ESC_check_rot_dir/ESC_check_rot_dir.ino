#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
int counter = 3;
int x = 0;
int i = 0;

void setup() {
  Serial.begin(9600);
    
    motA.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(8, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

}

void loop() {
  while (x < counter) {
    Serial.print(x);
    i = MIN_PULSE_LENGTH + 20;
    motA.writeMicroseconds(i);
    motB.writeMicroseconds(i);
    motC.writeMicroseconds(i);
    motD.writeMicroseconds(i);
    delay(200);
    x++;
  }
  motA.writeMicroseconds(MIN_PULSE_LENGTH);
  motB.writeMicroseconds(MIN_PULSE_LENGTH);
  motC.writeMicroseconds(MIN_PULSE_LENGTH);
  motD.writeMicroseconds(MIN_PULSE_LENGTH);
  delay(500);
}
