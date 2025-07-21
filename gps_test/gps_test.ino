#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>

// For NEO-6M, typical baud rate is 9600.
// If using software serial (e.g., not on hardware RX/TX pins 0,1):
SoftwareSerial gpsSerial(4, 3); // RX, TX (connect GPS TX to Arduino pin 4, GPS RX to Arduino pin 3)
LiquidCrystal_I2C lcd(0x27,20,4); // green SCL, blue SDA

TinyGPSPlus gps; // The TinyGPS++ object

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);    // Serial monitor baud rate
  gpsSerial.begin(9600); // GPS module baud rate
  Serial.println("Waiting for GPS data...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // A complete NMEA sentence has been processed
      if (gps.location.isValid()) {
        Serial.print("Latitude : ");
        lcd.setCursor(0,0);
        lcd.print("Lat: ");
        lcd.print(gps.location.lat(), 6);
        lcd.print("        "); // Clear rest of line if needed
        Serial.println(gps.location.lat(), 6); // 6 decimal places for precision
        Serial.print("Longitude: ");
        lcd.setCursor(0,1);
        lcd.print("Lon: ");
        lcd.print(gps.location.lng(), 6);
        lcd.print("        "); // Clear rest of line if needed
        Serial.println(gps.location.lng(), 6);
        Serial.print("Altitude : ");
        Serial.println(gps.altitude.meters(), 2);
        Serial.print("Satellites: ");
        Serial.println(gps.satellites.value());
        Serial.print("HDOP     : ");
        Serial.println(gps.hdop.value() / 100.0, 2); // HDOP is often reported as integer * 100
        Serial.print("Speed    : ");
        Serial.println(gps.speed.knots());
        Serial.print("Date     : ");
        Serial.print(gps.date.day());
        Serial.print("/");
        Serial.print(gps.date.month());
        Serial.print("/");
        Serial.println(gps.date.year());
        Serial.print("Time     : ");
        Serial.print(gps.time.hour());
        Serial.print(":");
        Serial.print(gps.time.minute());
        Serial.print(":");
        Serial.print(gps.time.second());
        Serial.println();
      } else {
        Serial.println("No GPS fix available.");
        lcd.setCursor(0,0);
        lcd.print("No GPS fix available.");
        lcd.setCursor(0,1);
        lcd.print("                     ");
      }
    }
  }
}