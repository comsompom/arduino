#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal_I2C.h>

// For NEO-6M, typical baud rate is 9600.
// If using software serial (e.g., not on hardware RX/TX pins 0,1):
SoftwareSerial gpsSerial(4, 3); // RX, TX (connect GPS TX to Arduino pin 4, GPS RX to Arduino pin 3)
LiquidCrystal_I2C lcd(0x27,20,4); // green SCL, blue SDA

TinyGPSPlus gps; // The TinyGPS++ object

// State management variables
enum State {
  WAITING_FOR_GPS,
  COUNTDOWN,
  TRACKING
};

State currentState = WAITING_FOR_GPS;
unsigned long countdownStartTime = 0;
unsigned long lastUpdateTime = 0;
const unsigned long COUNTDOWN_DURATION = 30000; // 30 seconds in milliseconds
const unsigned long UPDATE_INTERVAL = 1000; // Update every second

// Reference coordinates (to be set after countdown)
double refLatitude = 0;
double refLongitude = 0;
double refAltitude = 0;
bool referenceSet = false;

void setup() {
  lcd.init();
  lcd.backlight();
  Serial.begin(9600);    // Serial monitor baud rate
  gpsSerial.begin(9600); // GPS module baud rate
  Serial.println("Waiting for GPS data...");
  
  lcd.setCursor(0,0);
  lcd.print("Waiting for GPS...");
  lcd.setCursor(0,1);
  lcd.print("                    ");
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      // A complete NMEA sentence has been processed
      if (gps.location.isValid()) {
        handleGPSFix();
      } else {
        handleNoGPSFix();
      }
    }
  }
  
  // Handle state-specific updates
  handleStateUpdates();
}

void handleGPSFix() {
  switch (currentState) {
    case WAITING_FOR_GPS:
      // First GPS fix detected, start countdown
      currentState = COUNTDOWN;
      countdownStartTime = millis();
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Fixed GPS");
      Serial.println("GPS fix acquired! Starting 30-second countdown...");
      break;
      
    case COUNTDOWN:
      // Update countdown display
      updateCountdown();
      break;
      
    case TRACKING:
      // Update distance and altitude tracking
      updateTracking();
      break;
  }
}

void handleNoGPSFix() {
  if (currentState == WAITING_FOR_GPS) {
    lcd.setCursor(0,0);
    lcd.print("Waiting for GPS...");
    lcd.setCursor(0,1);
    lcd.print("                    ");
  }
  // In other states, we keep the current display
}

void handleStateUpdates() {
  unsigned long currentTime = millis();
  
  // Only update at regular intervals to avoid flickering
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    
    switch (currentState) {
      case COUNTDOWN:
        updateCountdown();
        break;
      case TRACKING:
        updateTracking();
        break;
    }
  }
}

void updateCountdown() {
  unsigned long elapsed = millis() - countdownStartTime;
  unsigned long remaining = COUNTDOWN_DURATION - elapsed;
  
  if (remaining <= 0) {
    // Countdown finished, set reference coordinates
    refLatitude = gps.location.lat();
    refLongitude = gps.location.lng();
    refAltitude = gps.altitude.meters();
    referenceSet = true;
    currentState = TRACKING;
    
    lcd.clear();
    Serial.println("Countdown finished! Reference coordinates set:");
    Serial.print("Lat: "); Serial.println(refLatitude, 6);
    Serial.print("Lon: "); Serial.println(refLongitude, 6);
    Serial.print("Alt: "); Serial.println(refAltitude, 2);
  } else {
    // Update countdown display
    unsigned long seconds = remaining / 1000;
    lcd.setCursor(0,1);
    lcd.print("Countdown: ");
    if (seconds < 10) lcd.print("0");
    lcd.print(seconds);
    lcd.print("s    ");
  }
}

void updateTracking() {
  if (!referenceSet) return;
  
  // Calculate distance from reference point
  double distance = TinyGPSPlus::distanceBetween(
    refLatitude, refLongitude,
    gps.location.lat(), gps.location.lng()
  );
  
  // Calculate altitude difference
  double altitudeDiff = gps.altitude.meters() - refAltitude;
  
  // Display distance on first line
  lcd.setCursor(0,0);
  lcd.print("Dist: ");
  if (distance < 1000) {
    lcd.print(distance, 1);
    lcd.print("m    ");
  } else {
    lcd.print(distance/1000, 2);
    lcd.print("km   ");
  }
  
  // Display altitude difference on second line
  lcd.setCursor(0,1);
  lcd.print("Alt: ");
  if (altitudeDiff >= 0) {
    lcd.print("+");
  }
  lcd.print(altitudeDiff, 1);
  lcd.print("m    ");
  
  // Also output to serial for debugging
  Serial.print("Distance: "); Serial.print(distance, 1); Serial.println("m");
  Serial.print("Altitude diff: "); Serial.print(altitudeDiff, 1); Serial.println("m");
}
