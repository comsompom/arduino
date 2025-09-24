// --- Constants (You can change these values) ---

// The Arduino pin connected to the relay module's IN pin
const int RELAY_PIN = 7;

// The Arduino Analog pin to read the input voltage from
const int SENSOR_PIN = A0;

// The voltage threshold. If the input voltage is above this, the relay turns ON.
// You can change this to any value, for example 2.0, 0.5, etc.
const float THRESHOLD_VOLTAGE = 2.5;


// --- Main Program (Do not change below unless you know what you are doing) ---

void setup() {
  // Initialize serial communication for debugging.
  // Open the Serial Monitor (Tools -> Serial Monitor) to see the voltage readings.
  Serial.begin(9600);

  // Set the relay pin as an output
  pinMode(RELAY_PIN, OUTPUT);

  // Ensure the relay is OFF when the Arduino starts up.
  // Note: Some relay modules are "active LOW", meaning LOW turns them ON.
  // If your relay turns on with this code, change HIGH to LOW and LOW to HIGH below.
  digitalWrite(RELAY_PIN, LOW); // Set relay to OFF state
  Serial.println("System Initialized. Relay is OFF.");
}

void loop() {
  // 1. Read the raw analog value (0-1023)
  int sensorValue = analogRead(SENSOR_PIN);

  // 2. Convert the analog value (0-1023) to a voltage (0-5V)
  // The Arduino's ADC reference is 5.0V and it has a 10-bit resolution (2^10 = 1024 values, so 0-1023)
  float voltage = sensorValue * (5.0 / 1023.0);

  // 3. Print the current voltage to the Serial Monitor for debugging
  Serial.print("Input Voltage: ");
  Serial.print(voltage);
  Serial.println(" V");

  // 4. Check if the voltage is above or below the threshold
  if (voltage >= THRESHOLD_VOLTAGE) {
    // If voltage is high, turn the relay ON
    digitalWrite(RELAY_PIN, HIGH);
    Serial.println("Threshold MET! ---> Relay ON");
  } else {
    // If voltage is low, turn the relay OFF
    digitalWrite(RELAY_PIN, LOW);
    Serial.println("Threshold NOT met. Relay OFF");
  }

  // Wait for a short moment before taking the next reading
  delay(100); // 500 milliseconds
}