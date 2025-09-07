/*******************************************************************************
 * USB Joystick to PPM Signal Generator - COMBINED SOLUTION
 *
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * - Turtle Beach VelocityOne Flightstick
 * - 3.5mm Mono Jack (for PPM output)
 *
 * WIRING:
 * - Arduino Pin 9 -> 3.5mm Jack TIP (PPM Signal)
 * - Arduino GND   -> 3.5mm Jack SLEEVE (Ground)
 *
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 *
 * DESCRIPTION:
 * This sketch combines the working joystick mapping from get_joystick.ino
 * with PPM signal generation to output RC control signals on pin 9.
 * The mapping is based on the corrected HID data analysis for the
 * Turtle Beach VelocityOne Flightstick.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- PPM Configuration --
#define PPM_PIN 9           // Digital pin for PPM output
#define NUM_CHANNELS 12     // Total number of channels (same as get_joystick.ino)
#define PPM_FRAME_LENGTH 22500 // Total PPM frame time in microseconds (22.5ms)
#define PPM_PULSE_LENGTH 300   // Pulse length in microseconds
#define PPM_CHANNEL_MIN 1000   // Minimum channel value (us)
#define PPM_CHANNEL_MAX 2000   // Maximum channel value (us)
#define PPM_CHANNEL_MID 1500   // Center channel value (us)

// Array to hold the values for each channel in microseconds
uint16_t ppm_channels[NUM_CHANNELS];

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel names for display (same as get_joystick.ino)
const char* channel_names[NUM_CHANNELS] = {
  "Throttle", "Elevator", "Aileron", "Button1", "Button2", "Button3",
  "Button4", "Button5", "Button6", "Button7", "Button8", "Fire"
};

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf); 
};

// Constructor for our parser
JoystickEvents::JoystickEvents() {}

// This function is called by the USB Host library every time the joystick sends a new data packet
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  // We only care about data packets with a length of 51, as per the log
  if (len == 51) {
    OnJoystickData(len, buf);
  }
}

// *** MAIN DATA PROCESSING FUNCTION - SAME AS get_joystick.ino ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // --- AXES ---
  // Based on the HID data pattern analysis:
  // Aileron (X-axis): bytes 1 & 2 (16-bit)
  // Elevator (Y-axis): bytes 3 & 4 (16-bit)
  // Throttle: bytes 10 & 11 (16-bit)

  // AILERON (bytes 1 & 2)
  uint16_t raw_aileron = (buf[2] << 8) | buf[1];
  ppm_channels[2] = map(raw_aileron, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // ELEVATOR (bytes 3 & 4)
  uint16_t raw_elevator = (buf[4] << 8) | buf[3];
  ppm_channels[1] = map(raw_elevator, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // THROTTLE (bytes 10-11)
  uint16_t raw_throttle = (buf[11] << 8) | buf[10];
  ppm_channels[0] = map(raw_throttle, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);

  // --- BUTTONS ---
  // Buttons B1-B8 are a bitmask in byte 18
  // Fire button is a bitmask in byte 20

  // BUTTONS B1-B8 (byte 18)
  uint8_t button_byte = buf[18];
  for (int i = 0; i < 8; i++) {
    bool button_state = (button_byte & (1 << i)) != 0;
    // Map boolean state to 1000 (released) or 2000 (pressed)
    ppm_channels[i + 3] = button_state ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;
  }
  
  // FIRE BUTTON (byte 20, bit 1)
  bool fire_pressed = (buf[20] & 0x02) != 0;
  ppm_channels[11] = fire_pressed ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;

  // Ensure all values are within valid PPM range
  for (int i = 0; i < NUM_CHANNELS; i++) {
    ppm_channels[i] = constrain(ppm_channels[i], PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);
  }

  // Display channel changes on Serial Monitor
  for (int i = 0; i < NUM_CHANNELS; i++) {
    static uint16_t previous_values[NUM_CHANNELS] = {0};
    if (ppm_channels[i] != previous_values[i]) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(channel_names[i]);
      Serial.print(" -> ");
      Serial.print(ppm_channels[i]);
      Serial.println("us");
      previous_values[i] = ppm_channels[i];
    }
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect
  
  Serial.println("\n=== USB Joystick to PPM Signal Generator ===");
  Serial.println("Device: Turtle Beach VelocityOne Flightstick");
  Serial.println("PPM Output: Pin 9");
  Serial.println("Waiting for joystick connection...");

  // Initialize all PPM channels to safe values
  for (int i = 0; i < 3; i++) { // Axes
    ppm_channels[i] = PPM_CHANNEL_MID;
  }
  for (int i = 3; i < NUM_CHANNELS; i++) { // Buttons
    ppm_channels[i] = PPM_CHANNEL_MIN;
  }
  
  // Set up PPM output pin
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, LOW);

  // Initialize the USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed! Halting.");
    while (1);
  }
  Serial.println("USB Host Shield Initialized.");

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);

  Serial.println("\n=== PPM Signal Generator Ready ===");
  Serial.println("Move joystick and press buttons to generate PPM signals.");
  Serial.println("PPM output active on Pin 9");
  Serial.println("======================================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running
  Usb.Task();

  // Generate PPM signal based on current channel values
  generatePpmSignal();
}

// PPM Signal Generation Function (from joystick_taranis.ino)
void generatePpmSignal() {
  static unsigned long last_frame_time = 0;
  unsigned long current_time = micros();

  // Wait until it's time to start the next PPM frame
  if (current_time - last_frame_time >= PPM_FRAME_LENGTH) {
    last_frame_time = current_time;
    
    unsigned long frame_start_time = micros();
    unsigned long cumulative_time = 0;

    for (int i = 0; i < NUM_CHANNELS; i++) {
      // 1. Start with a short low pulse (separator)
      digitalWrite(PPM_PIN, LOW);
      delayMicroseconds(PPM_PULSE_LENGTH);

      // 2. High pulse representing the channel value
      digitalWrite(PPM_PIN, HIGH);
      delayMicroseconds(ppm_channels[i] - PPM_PULSE_LENGTH);
      
      cumulative_time += ppm_channels[i];
    }

    // 3. Final low pulse before the sync period
    digitalWrite(PPM_PIN, LOW);
    delayMicroseconds(PPM_PULSE_LENGTH);
    cumulative_time += PPM_PULSE_LENGTH;

    // 4. Sync pulse (the remainder of the frame)
    // No need for code here, the signal just stays low until the next frame starts
  }
}
