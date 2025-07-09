#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// For NEO-6M, typical baud rate is 9600.
// If using software serial (e.g., not on hardware RX/TX pins 0,1):
SoftwareSerial gpsSerial(4, 3); // RX, TX (connect GPS TX to Arduino pin 2, GPS RX to Arduino pin 3)

TinyGPSPlus gps; // The TinyGPS++ object

void setup() {
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
        Serial.println(gps.location.lat(), 6); // 6 decimal places for precision
        Serial.print("Longitude: ");
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
      }
    }
  }
}