#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
char data;

void setup() {
  Serial.begin(57600);
  
  motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

}

void loop() {
  if (Serial.available()) {
      data = Serial.read();

      switch (data) {
          // 0
          case 48 : Serial.println("Sending minimum throttle");
                    motA.writeMicroseconds(MIN_PULSE_LENGTH);
                    motB.writeMicroseconds(MIN_PULSE_LENGTH);
                    motC.writeMicroseconds(MIN_PULSE_LENGTH);
                    motD.writeMicroseconds(MIN_PULSE_LENGTH);
          break;

          // 1
          case 49 : Serial.println("Sending maximum throttle");
                    motA.writeMicroseconds(MAX_PULSE_LENGTH);
                    motB.writeMicroseconds(MAX_PULSE_LENGTH);
                    motC.writeMicroseconds(MAX_PULSE_LENGTH);
                    motD.writeMicroseconds(MAX_PULSE_LENGTH);
          break;

          // 2
          case 50 : Serial.print("Running test in 3");
                    delay(1000);
                    Serial.print(" 2");
                    delay(1000);
                    Serial.println(" 1...");
                    delay(1000);
                    test();
          break;
      }
  }
    
}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{
    for (int i = MIN_PULSE_LENGTH; i <= 1300; i += 20) {
        Serial.print("Pulse length = ");
        Serial.println(i);
        
        motA.writeMicroseconds(i);
        motB.writeMicroseconds(i);
        motC.writeMicroseconds(i);
        motD.writeMicroseconds(i);
        
        delay(200);
    }

    Serial.println("STOP");
    motA.writeMicroseconds(MIN_PULSE_LENGTH);
    motB.writeMicroseconds(MIN_PULSE_LENGTH);
    motC.writeMicroseconds(MIN_PULSE_LENGTH);
    motD.writeMicroseconds(MIN_PULSE_LENGTH);
}
