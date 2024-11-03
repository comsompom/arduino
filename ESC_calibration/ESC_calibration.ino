/**
 * Usage, according to documentation(https://www.firediy.fr/files/drone/HW-01-V4.pdf) : 
 *     1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode
 *     2. Power up your ESCs. You must hear "beep1 beep2 beep3" tones meaning the power supply is OK
 *     3. After 2sec, "beep beep" tone emits, meaning the throttle highest point has been correctly confirmed
 *     4. Type 0 to send min throttle
 *     5. Several "beep" tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)
 *     6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed
 *     7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them
 */
 // ESC - should be calibrated first. For this:
 // 1. Connect ESC to the receiver FSIA6B
 // 2. power off the ESC
 // 3. Power on the transmitter FSIA6B
 // 4. Move Throtle to the highest position
 // 5. Power ON ESC. and wait beeps finished
 // 6. Move throtle to the down position and wait ESC finished beep
 // 7. Power Off FSIA6b Transmitter
 // 8. Power On FSIA6b Transmitter and move throttle little the ESC schould send the signal and motor start.
 // 9. Move throttle down motor should stop
 // 10. Power OFF ESC, Power OFF Transmitter. Calibration complete
// ---------------------------------------------------------------------------
#include <Servo.h>
// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
#define TEST_PULSE_LENGTH 1400 // Set the test pulse in ms
// ---------------------------------------------------------------------------
Servo mot1, mot2, mot3, mot4;
char data;
int CUR_PULSE_LENGTH;
// ---------------------------------------------------------------------------

/**
 * Initialisation routine
 */
void setup() {
    Serial.begin(57600);
    
    mot1.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot2.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot3.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    mot4.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);

    CUR_PULSE_LENGTH = MIN_PULSE_LENGTH;
    
    displayInstructions();
}

/**
 * Main function
 */
void loop() {
    if (Serial.available()) {
        Serial.println(CUR_PULSE_LENGTH);
        data = Serial.read();

        switch (data) {
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      CUR_PULSE_LENGTH = MIN_PULSE_LENGTH;
                      mot1.writeMicroseconds(MIN_PULSE_LENGTH);
                      mot2.writeMicroseconds(MIN_PULSE_LENGTH);
                      mot3.writeMicroseconds(MIN_PULSE_LENGTH);
                      mot4.writeMicroseconds(MIN_PULSE_LENGTH);
            break;

            // 1
            case 49 : Serial.println("INCREASE 100 throttle");
                      Serial.println(" 1...");
                      delay(1000);
                      send_throttle_up();
            break;

            // 2
            case 50 : Serial.println("DECREASE 100 throttle");
                      Serial.println(" 1...");
                      delay(1000);
                      send_throttle_down();
            break;

            // 3
            case 51 : Serial.print("Running test in 3");
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

void send_throttle_up(){
    for (int i = 0; i <= 50; i += 10) {
        CUR_PULSE_LENGTH += 10;
        Serial.println(CUR_PULSE_LENGTH);

        mot1.writeMicroseconds(CUR_PULSE_LENGTH + 10);
        mot2.writeMicroseconds(CUR_PULSE_LENGTH);
        mot3.writeMicroseconds(CUR_PULSE_LENGTH);
        mot4.writeMicroseconds(CUR_PULSE_LENGTH + 10);
            
        delay(200);
    }
}

void send_throttle_down(){
    for (int i = 0; i <= 50; i += 10) {
        CUR_PULSE_LENGTH -= 10;
        Serial.println(CUR_PULSE_LENGTH);

        mot1.writeMicroseconds(CUR_PULSE_LENGTH);
        mot2.writeMicroseconds(CUR_PULSE_LENGTH);
        mot3.writeMicroseconds(CUR_PULSE_LENGTH);
        mot4.writeMicroseconds(CUR_PULSE_LENGTH);
            
        delay(200);
    }
}

/**
 * Test function: send min throttle to max throttle to each ESC.
 */
void test()
{

    for (int i = MIN_PULSE_LENGTH; i <= TEST_PULSE_LENGTH; i += 40) {
        Serial.print("Pulse length = ");
        if (i > TEST_PULSE_LENGTH) i = TEST_PULSE_LENGTH;
        Serial.println(i);
        
        mot1.writeMicroseconds(i);
        mot2.writeMicroseconds(i);
        mot3.writeMicroseconds(i);
        mot4.writeMicroseconds(i);
        
        delay(200);
    }

    for (int i = TEST_PULSE_LENGTH; i >= MIN_PULSE_LENGTH; i -= 40) {
        Serial.print("Pulse length = ");
        if (i < MIN_PULSE_LENGTH) i = MIN_PULSE_LENGTH;
        Serial.println(i);
        
        mot1.writeMicroseconds(i);
        mot2.writeMicroseconds(i);
        mot3.writeMicroseconds(i);
        mot4.writeMicroseconds(i);
        
        delay(200);
    }

    Serial.println("STOP");
    mot1.writeMicroseconds(MIN_PULSE_LENGTH);
    mot2.writeMicroseconds(MIN_PULSE_LENGTH);
    mot3.writeMicroseconds(MIN_PULSE_LENGTH);
    mot4.writeMicroseconds(MIN_PULSE_LENGTH);
}

/**
 * Displays instructions to user
 */
void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("\t0 : Send min throttle");
    Serial.println("\t1 : INCREASE throttle");
    Serial.println("\t2 : DECREASE function");
    Serial.println("\t3 : Send Test function\n");
}
