/*
  Arduino Uno Sketch to control a Common Anode RGBY LED Strip

  This sketch demonstrates how to control the color and brightness of the LED strip
  by using Pulse Width Modulation (PWM).
*/

// Define the Arduino pins connected to the LED strip's color channels
const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;
const int YELLOW_PIN = 6;

void setup() {
  // Set all the LED pins as outputs
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  pinMode(YELLOW_PIN, OUTPUT);

  // Initialize all LEDs to the "off" state.
  // For Common Anode, HIGH is off and LOW is on.
  digitalWrite(RED_PIN, HIGH);
  digitalWrite(GREEN_PIN, HIGH);
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(YELLOW_PIN, HIGH);

  Serial.begin(9600);
  Serial.println("RGBY LED Strip Controller Initialized");
}

void loop() {
  // --- Part 1: Basic Color Test ---
  Serial.println("Testing Red...");
  setColor(255, 0, 0, 0); // Full Red
  delay(1000);

  Serial.println("Testing Green...");
  setColor(0, 255, 0, 0); // Full Green
  delay(1000);

  Serial.println("Testing Blue...");
  setColor(0, 0, 255, 0); // Full Blue
  delay(1000);

  Serial.println("Testing Yellow...");
  setColor(0, 0, 0, 255); // Full Yellow
  delay(1000);

  // --- Part 2: Color Mixing Test ---
  Serial.println("Testing Purple (Red + Blue)...");
  setColor(255, 0, 255, 0);
  delay(1500);

  Serial.println("Testing Cyan (Green + Blue)...");
  setColor(0, 255, 255, 0);
  delay(1500);

  Serial.println("Testing White (Red + Green + Blue)...");
  setColor(255, 255, 255, 0);
  delay(1500);

  // --- Part 3: Rainbow Fade Effect ---
  Serial.println("Starting rainbow fade effect...");
  // Fade from Red to Green
  for (int i = 0; i < 255; i++) {
    setColor(255 - i, i, 0, 0);
    delay(10);
  }
  // Fade from Green to Blue
  for (int i = 0; i < 255; i++) {
    setColor(0, 255 - i, i, 0);
    delay(10);
  }
  // Fade from Blue to Red
  for (int i = 0; i < 255; i++) {
    setColor(i, 0, 255 - i, 0);
    delay(10);
  }
}

/**
 * @brief Sets the color of the RGBY LED strip.
 * 
 * @param red   Brightness of the red channel (0-255)
 * @param green Brightness of the green channel (0-255)
 * @param blue  Brightness of the blue channel (0-255)
 * @param yellow Brightness of the yellow channel (0-255)
 */
void setColor(int red, int green, int blue, int yellow) {
  // IMPORTANT: Because this is a Common Anode LED strip, the logic is inverted.
  // 0 means full brightness, and 255 means off.
  // We subtract the desired brightness from 255 to get the correct PWM value.
  analogWrite(RED_PIN, 255 - red);
  analogWrite(GREEN_PIN, 255 - green);
  analogWrite(BLUE_PIN, 255 - blue);
  analogWrite(YELLOW_PIN, 255 - yellow);
}
