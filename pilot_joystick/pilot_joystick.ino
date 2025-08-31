/*******************************************************************************
 * Turtle Beach VelocityOne Flightstick to Taranis QX7 PPM Translator
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * - 3.5mm Mono Jack
 * 
 * WIRING:
 * - Arduino Pin 9 -> 3.5mm Jack TIP (PPM Signal)
 * - Arduino GND   -> 3.5mm Jack SLEEVE (Ground)
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Reads USB HID data from the Flightstick, maps the axes to RC channels,
 * and generates a PPM signal for the Taranis QX7 trainer port.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- PPM Configuration --
#define PPM_PIN 9           // Digital pin for PPM output.
#define NUM_CHANNELS 20      // Total number of channels in the PPM stream.
#define PPM_FRAME_LENGTH 22500 // Total PPM frame time in microseconds (22.5ms).
#define PPM_PULSE_LENGTH 300   // Pulse length in microseconds.
#define PPM_CHANNEL_MIN 1000   // Minimum channel value (us).
#define PPM_CHANNEL_MAX 2000   // Maximum channel value (us).
#define PPM_CHANNEL_MID 1500   // Center channel value (us).

// Array to hold the values for each channel in microseconds.
unsigned int ppm_channels[NUM_CHANNELS];

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

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

// This function is called by the USB Host library every time the joystick sends a new data packet.
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  OnJoystickData(len, buf);
  Serial.print("Joystick Data: ");
}

// *** THIS IS THE MOST IMPORTANT FUNCTION TO CUSTOMIZE ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // This is where you translate the raw joystick data buffer (buf)
  // into your ppm_channels array.
  
  // --- STEP 1: DEBUGGING - Find your axis values ---
  // Uncomment the following block to print the raw data from the joystick.
  // Open the Serial Monitor at 115200 baud, move one axis at a time,
  // and observe which bytes in the output change.
  
  Serial.print("HID Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  

  // --- STEP 2: MAPPING - Update this section based on your findings ---
  // The following is an EXAMPLE based on a typical joystick.
  // Your VelocityOne Flightstick might be different!
  // Joysticks often use 10-bit values (0-1023) or 16-bit values.
  // Let's assume 10-bit values split across two bytes.

  // Example: Aileron (X-Axis) might be in bytes 0 and 1
  int aileron_raw = (buf[1] << 8) | buf[0]; // Combine two bytes for a 16-bit value
  ppm_channels[0] = map(aileron_raw, 0, 1023, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX); // CH1: Aileron

  // Example: Elevator (Y-Axis) might be in bytes 2 and 3
  int elevator_raw = (buf[3] << 8) | buf[2];
  ppm_channels[1] = map(elevator_raw, 0, 1023, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX); // CH2: Elevator
  // ppm_channels[1] = map(elevator_raw, 1023, 0, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX); // Use this if axis is reversed

  // Example: Throttle might be in bytes 4 and 5
  int throttle_raw = (buf[5] << 8) | buf[4];
  ppm_channels[2] = map(throttle_raw, 0, 1023, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX); // CH3: Throttle
  
  // Example: Rudder (Twist) might be in bytes 6 and 7
  int rudder_raw = (buf[7] << 8) | buf[6];
  ppm_channels[3] = map(rudder_raw, 0, 1023, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX); // CH4: Rudder

  // Make sure values are within the valid PPM range
  for(int i=0; i<4; i++) {
    ppm_channels[i] = constrain(ppm_channels[i], PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;


void setup() {
  Serial.begin(115200);
  Serial.println("Starting VelocityOne to PPM Translator");

  // Initialize all PPM channels to a neutral/safe state (1500us for sticks, 1000us for throttle)
  ppm_channels[0] = PPM_CHANNEL_MID; // Aileron
  ppm_channels[1] = PPM_CHANNEL_MID; // Elevator
  ppm_channels[2] = PPM_CHANNEL_MIN; // Throttle (low)
  ppm_channels[3] = PPM_CHANNEL_MID; // Rudder
  for (int i = 4; i < NUM_CHANNELS; i++) {
    ppm_channels[i] = PPM_CHANNEL_MIN; // Other channels low
  }
  
  pinMode(PPM_PIN, OUTPUT);
  digitalWrite(PPM_PIN, LOW); // Start with a low signal

  // Initialize the USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
    while (1); // Halt
  }
  Serial.println("USB Host Shield Initialized");

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);
}


void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // This function generates the PPM signal based on the current ppm_channels array.
  generatePpmSignal();
}


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
      // Serial.println(i);
      // Serial.println(ppm_channels[i]);
    }

    // 3. Final low pulse before the sync period
    digitalWrite(PPM_PIN, LOW);
    delayMicroseconds(PPM_PULSE_LENGTH);
    cumulative_time += PPM_PULSE_LENGTH;

    // 4. Sync pulse (the remainder of the frame)
    // No need for code here, the signal just stays low until the next frame starts.
  }
}