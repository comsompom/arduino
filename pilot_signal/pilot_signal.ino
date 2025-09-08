/*******************************************************************************
 * USB Joystick to PPM Signal Generator - FIXED & SMOOTHED SOLUTION
 *
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * - Turtle Beach VelocityOne Flightstick
 *
 * DESCRIPTION:
 * This sketch fixes the incorrect throttle mapping and jumping values by:
 * 1. Using calibrated MIN/MAX values for the throttle axis.
 * 2. Implementing a smoothing algorithm (low-pass filter) to eliminate jitter.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- PPM Configuration --
#define PPM_PIN 2           // Digital pin for PPM output
#define NUM_CHANNELS 12     // Total number of channels
#define PPM_FRAME_LENGTH 22500 // Total PPM frame time in microseconds (22.5ms)
#define PPM_PULSE_LENGTH 300   // Pulse length in microseconds
#define PPM_CHANNEL_MIN 1000   // Minimum channel value (us)
#define PPM_CHANNEL_MAX 2000   // Maximum channel value (us)
#define PPM_CHANNEL_MID 1500   // Center channel value (us)

// ******************** CALIBRATION & SMOOTHING ********************
// STEP 1: Find your throttle's real min/max raw values.
//   - Set FIND_THROTTLE_RANGE to true and upload.
//   - Open the Serial Monitor.
//   - Move the throttle slowly from its absolute minimum to its absolute maximum.
//   - Note the lowest and highest "Raw Throttle" values printed.
//   - Update the THROTTLE_RAW_MIN and THROTTLE_RAW_MAX values below with what you found.
//   - Set FIND_THROTTLE_RANGE back to false and re-upload.
#define FIND_THROTTLE_RANGE false // Set to true ONLY for calibration

const uint16_t THROTTLE_RAW_MIN = 127;    // <--- REPLACE WITH YOUR MIN VALUE (e.g., 250)
const uint16_t THROTTLE_RAW_MAX = 65407; // <--- REPLACE WITH YOUR MAX VALUE (e.g., 65200)

// STEP 2: Adjust smoothing. A lower value means more smoothing but slower response.
// A good starting point is 0.05 to 0.1.
#define THROTTLE_SMOOTHING_FACTOR 0.07f

// Array to hold the values for each channel in microseconds
uint16_t ppm_channels[NUM_CHANNELS];

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel names for display
const char* channel_names[NUM_CHANNELS] = {
  "Fire", "Throttle", "Elevator", "Aileron", "Button1", "Button2",
  "Button3", "Button4", "Button5", "Button6", "Button7", "Button8"
};

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
};

JoystickEvents::JoystickEvents() {}

void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  if (len == 51) {
    OnJoystickData(len, buf);
  }
}

// *** MAIN DATA PROCESSING FUNCTION - WITH FIXES ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // --- AXES ---
  // AILERON (bytes 1 & 2) -> Channel 3 (mapped to Aileron)
  uint16_t raw_aileron = (buf[2] << 8) | buf[1];
  ppm_channels[3] = map(raw_aileron, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // ELEVATOR (bytes 3 & 4) -> Channel 2 (mapped to Elevator)
  uint16_t raw_elevator = (buf[4] << 8) | buf[3];
  ppm_channels[2] = map(raw_elevator, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // === THROTTLE FIX START ===
  
  // 1. Read the raw throttle value from bytes 10 & 11
  uint16_t raw_throttle = (buf[11] << 8) | buf[10];

  // 2. Map the raw value using your CALIBRATED min and max
  //    Note: The throttle is often reversed (max throttle = min raw value).
  //    If your throttle is inverted, swap THROTTLE_RAW_MIN and THROTTLE_RAW_MAX in the map() call.
  //    e.g., map(raw_throttle, THROTTLE_RAW_MAX, THROTTLE_RAW_MIN, ...)
  long mapped_throttle = map(raw_throttle, THROTTLE_RAW_MIN, THROTTLE_RAW_MAX, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // 3. Apply smoothing to eliminate jitter and create a stable output
  static float smoothed_throttle = PPM_CHANNEL_MID; // Start at the middle value
  smoothed_throttle = (THROTTLE_SMOOTHING_FACTOR * mapped_throttle) + ((1.0f - THROTTLE_SMOOTHING_FACTOR) * smoothed_throttle);

  // 4. Assign the final, smoothed value to the PPM channel
  //    The static variable 'smoothed_throttle' holds its value between calls,
  //    ensuring the value does not jump back to a default.
  ppm_channels[1] = (uint16_t)smoothed_throttle;

  // --- THROTTLE FIX END ---

  // --- BUTTONS ---
  // Buttons B1-B8 (byte 18) -> Channels 4-11
  uint8_t button_byte = buf[18];
  for (int i = 0; i < 8; i++) {
    bool button_state = (button_byte & (1 << i)) != 0;
    ppm_channels[i + 4] = button_state ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;
  }

  // FIRE BUTTON (byte 20, bit 1) -> Channel 0
  bool fire_pressed = (buf[20] & 0x02) != 0;
  ppm_channels[0] = fire_pressed ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;

  // Ensure all values are within the valid PPM range
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ppm_channels[i] = constrain(ppm_channels[i], PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);
  }
  
  // --- DEBUGGING OUTPUT ---
  #if FIND_THROTTLE_RANGE
    // This code runs ONLY when you are calibrating
    static unsigned long last_debug_time = 0;
    if (millis() - last_debug_time > 100) { // Print every 100ms
      Serial.print("Move throttle to find range -> Raw Throttle: ");
      Serial.println(raw_throttle);
      last_debug_time = millis();
    }
  #else
    // This code runs during normal operation
    static uint16_t previous_values[NUM_CHANNELS] = {0};
    for (int i = 0; i < NUM_CHANNELS; i++) {
      // Print only when a channel value changes to avoid spamming the monitor
      if (abs(ppm_channels[i] - previous_values[i]) > 2) { // Threshold of 2 to ignore tiny smoothed changes
        if (i == 1) { // Special print for throttle to see all values
          Serial.print("CH2: Throttle -> Raw: ");
          Serial.print(raw_throttle);
          Serial.print(" | Mapped: ");
          Serial.print(mapped_throttle);
          Serial.print(" | Smoothed PPM: ");
          Serial.print(ppm_channels[i]);
          Serial.println("us");
        } else {
          Serial.print("CH");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(channel_names[i]);
          Serial.print(" -> ");
          Serial.print(ppm_channels[i]);
          Serial.println("us");
        }
        previous_values[i] = ppm_channels[i];
      }
    }
  #endif
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("\n=== USB Joystick to PPM Signal Generator (FIXED) ===");
  #if FIND_THROTTLE_RANGE
    Serial.println("\n!!! CALIBRATION MODE ACTIVE !!!");
    Serial.println("Move throttle slowly from MIN to MAX to find its range.");
    Serial.println("Update THROTTLE_RAW_MIN/MAX in the code, then set FIND_THROTTLE_RANGE to false.");
    Serial.println("======================================================");
  #else
    Serial.println("Device: Turtle Beach VelocityOne Flightstick");
    Serial.println("PPM Output: Pin 2");
    Serial.println("Waiting for joystick connection...");
  #endif

  // Initialize all PPM channels to safe values
  ppm_channels[0] = PPM_CHANNEL_MIN;  // Fire button (released)
  ppm_channels[1] = PPM_CHANNEL_MIN;  // Throttle at minimum
  ppm_channels[2] = PPM_CHANNEL_MID;  // Elevator at center
  ppm_channels[3] = PPM_CHANNEL_MID;  // Aileron at center
  for (int i = 4; i < NUM_CHANNELS; i++) {
    ppm_channels[i] = PPM_CHANNEL_MIN;
  }

  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, LOW);

  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed! Halting.");
    while (1);
  }
  
  Hid.SetReportParser(0, &JoyEvents);
}

void loop() {
  Usb.Task();
  generatePpmSignal();
}

void generatePpmSignal() {
  static unsigned long last_frame_time = 0;
  if (micros() - last_frame_time >= PPM_FRAME_LENGTH) {
    last_frame_time = micros();

    for (int i = 0; i < NUM_CHANNELS; i++) {
      digitalWrite(PPM_PIN, LOW);
      delayMicroseconds(PPM_PULSE_LENGTH);
      digitalWrite(PPM_PIN, HIGH);
      delayMicroseconds(ppm_channels[i] - PPM_PULSE_LENGTH);
    }

    digitalWrite(PPM_PIN, LOW);
    // The rest of the frame is the sync pulse (low state)
  }
}