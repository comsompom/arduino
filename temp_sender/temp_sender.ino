#include <Wire.h>
#include <Adafruit_BMP085.h>

// Create an object for the BMP085 sensor
Adafruit_BMP085 bmp;

void setup() {
  // Start the serial communication for debugging
  Serial.begin(9600);
  delay(1000); // Give serial time to initialize
  
  Serial.println("=== BMP085 Temperature Sensor Test ===");
  Serial.println("Initializing BMP085 sensor...");

  // Initialize I2C
  Wire.begin();
  
  // Initialize the sensor
  if (!bmp.begin()) {
    Serial.println("ERROR: Could not find a valid BMP085 sensor!");
    Serial.println("Check wiring:");
    Serial.println("1. VCC connected to Arduino 3.3V or 5V");
    Serial.println("2. GND connected to Arduino GND");
    Serial.println("3. SCL connected to Arduino A5");
    Serial.println("4. SDA connected to Arduino A4");
    Serial.println();
    Serial.println("Press RESET button to try again...");
    while (1) {} // Halt the program if sensor is not found
  }
  
  Serial.println("BMP085 sensor initialized successfully!");
  Serial.println("Reading temperature every 2 seconds...");
  Serial.println();
}

void loop() {
  // Read the temperature from the sensor
  float temperature = bmp.readTemperature();
  
  // Check if temperature reading is valid
  if (isnan(temperature)) {
    Serial.println("ERROR: Invalid temperature reading!");
    delay(1000);
    return;
  }

  // Print the temperature to the Serial Monitor
  Serial.print("Temperature = ");
  Serial.print(temperature, 2); // Show 2 decimal places
  Serial.println(" °C");

  // You can also read pressure and altitude if you want!
  /*
  float pressure = bmp.readPressure();
  float altitude = bmp.readAltitude();
  
  if (!isnan(pressure)) {
    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" Pa");
  }
  
  if (!isnan(altitude)) {
    Serial.print("Altitude = ");
    Serial.print(altitude, 2);
    Serial.println(" meters");
  }
  */

  Serial.println(); // Add a blank line for readability

  // Wait for 2 seconds before the next reading
  delay(2000);
}
