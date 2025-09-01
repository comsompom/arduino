/*******************************************************************************
 * USB Joystick Complete Control Mapper
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * - Turtle Beach VelocityOne Flightstick
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Maps all joystick controls based on HID data patterns and shows
 * channel changes with channel number and name
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 12      // Total channels for all controls
#define DEBUG_MODE 0         // Set to 1 for detailed debugging

// Array to hold the values for each channel
uint16_t channel_values[NUM_CHANNELS];
uint16_t previous_channel_values[NUM_CHANNELS];

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel names for display
const char* channel_names[NUM_CHANNELS] = {
  "Throttle", "Elevator", "Aileron", "Button1", "Button2", "Button3",
  "Button4", "Button5", "Button6", "Button7", "Button8", "Fire"
};

// Button states tracking
bool button_states[9] = {false, false, false, false, false, false, false, false, false};
bool previous_button_states[9] = {false, false, false, false, false, false, false, false, false};

// Control states
uint16_t throttle_raw = 0;
uint16_t elevator_raw = 0;
uint16_t aileron_raw = 0;
bool fire_button_state = false;

int data_packet_count = 0;

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
  void parseThrottle(uint8_t len, uint8_t *buf);
  void parseElevator(uint8_t len, uint8_t *buf);
  void parseAileron(uint8_t len, uint8_t *buf);
  void parseButtons(uint8_t len, uint8_t *buf);
  void parseFireButton(uint8_t len, uint8_t *buf);
};

// Constructor for our parser
JoystickEvents::JoystickEvents() {}

// This function is called by the USB Host library every time the joystick sends a new data packet.
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  OnJoystickData(len, buf);
}

void JoystickEvents::parseThrottle(uint8_t len, uint8_t *buf) {
  // Based on the HID data pattern:
  // Throttle data is in bytes 8-9 (16-bit value)
  // Throttle at zero:     bytes 8-9 = 0xE0 0x80
  // Throttle at 100%:     bytes 8-9 = 0xFF 0xFF
  // Throttle at 50%:      bytes 8-9 = 0xC0 0x73
  // Throttle 65-78%:      bytes 8-9 = 0x40 0xD8
  // Throttle 18-30%:      bytes 8-9 = 0xC0 0x43
  
  if (len > 9) {
    // Combine bytes 8-9 into 16-bit throttle value
    uint16_t raw_throttle = (buf[9] << 8) | buf[8];
    
    // Check if this is a valid throttle reading
    if (raw_throttle >= 0x80E0 && raw_throttle <= 0xFFFF) {
      throttle_raw = raw_throttle;
      
      // Map throttle to channel value (1000-2000us)
      // 0x80E0 (zero) -> 1000us
      // 0xFFFF (100%) -> 2000us
      uint16_t mapped_throttle = map(raw_throttle, 0x80E0, 0xFFFF, 1000, 2000);
      channel_values[0] = mapped_throttle;
    }
  }
}

void JoystickEvents::parseElevator(uint8_t len, uint8_t *buf) {
  // Based on the HID data pattern:
  // Elevator data is in bytes 0-1 (16-bit value)
  // Elevator mid-down:  bytes 0-1 = 0x20 0x82
  // Elevator down:      bytes 0-1 = 0x60 0x81
  // Elevator mid-up:    bytes 0-1 = 0xA0 0x89
  // Elevator up:        bytes 0-1 = 0x40 0x85
  
  if (len > 1) {
    // Combine bytes 0-1 into 16-bit elevator value
    uint16_t raw_elevator = (buf[1] << 8) | buf[0];
    
    // Check if this is a valid elevator reading
    if (raw_elevator >= 0x0000 && raw_elevator <= 0xFFFF) {
      elevator_raw = raw_elevator;
      
      // Map elevator to channel value (1000-2000us)
      // 0x0000 (down) -> 1000us
      // 0xFFFF (up) -> 2000us
      uint16_t mapped_elevator = map(raw_elevator, 0x0000, 0xFFFF, 1000, 2000);
      channel_values[1] = mapped_elevator;
    }
  }
}

void JoystickEvents::parseAileron(uint8_t len, uint8_t *buf) {
  // Based on the HID data pattern:
  // Aileron data is in bytes 0-1 (16-bit value)
  // Aileron mid-left:   bytes 0-1 = 0x40 0x4C
  // Aileron left:       bytes 0-1 = 0x00 0x00
  // Aileron mid-right:  bytes 0-1 = 0x60 0xC9
  // Aileron right:      bytes 0-1 = 0xFF 0xFF
  
  if (len > 1) {
    // Combine bytes 0-1 into 16-bit aileron value
    uint16_t raw_aileron = (buf[1] << 8) | buf[0];
    
    // Check if this is a valid aileron reading
    if (raw_aileron >= 0x0000 && raw_aileron <= 0xFFFF) {
      aileron_raw = raw_aileron;
      
      // Map aileron to channel value (1000-2000us)
      // 0x0000 (left) -> 1000us
      // 0xFFFF (right) -> 2000us
      uint16_t mapped_aileron = map(raw_aileron, 0x0000, 0xFFFF, 1000, 2000);
      channel_values[2] = mapped_aileron;
    }
  }
}

void JoystickEvents::parseButtons(uint8_t len, uint8_t *buf) {
  // Based on the HID data pattern:
  // Button data is in byte 18 (bit field)
  // Button B8: byte 18 = 0x80 (bit 7)
  // Button B7: byte 18 = 0x40 (bit 6)
  // Button B6: byte 18 = 0x20 (bit 5)
  // Button B5: byte 18 = 0x10 (bit 4)
  // Button B4: byte 18 = 0x08 (bit 3)
  // Button B3: byte 18 = 0x04 (bit 2)
  // Button B2: byte 18 = 0x02 (bit 1)
  // Button B1: byte 18 = 0x01 (bit 0)
  
  if (len > 18) {
    uint8_t button_byte = buf[18];
    
    // Parse each button bit
    button_states[0] = (button_byte & 0x01) != 0;  // B1
    button_states[1] = (button_byte & 0x02) != 0;  // B2
    button_states[2] = (button_byte & 0x04) != 0;  // B3
    button_states[3] = (button_byte & 0x08) != 0;  // B4
    button_states[4] = (button_byte & 0x10) != 0;  // B5
    button_states[5] = (button_byte & 0x20) != 0;  // B6
    button_states[6] = (button_byte & 0x40) != 0;  // B7
    button_states[7] = (button_byte & 0x80) != 0;  // B8
    
    // Update channel values for buttons (CH4-CH11)
    for (int i = 0; i < 8; i++) {
      channel_values[i + 3] = button_states[i] ? 2000 : 1000;
    }
  }
}

void JoystickEvents::parseFireButton(uint8_t len, uint8_t *buf) {
  // Based on the HID data pattern:
  // Fire button data is in byte 20
  // Fire button pressed:  byte 20 = 0x02
  // Fire button released: byte 20 = 0x00
  
  if (len > 20) {
    bool fire_pressed = (buf[20] & 0x02) != 0;
    fire_button_state = fire_pressed;
    
    // Update channel value for fire button (CH12)
    channel_values[11] = fire_pressed ? 2000 : 1000;
  }
}

// *** MAIN DATA PROCESSING FUNCTION ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  data_packet_count++;
  
  // Parse all controls
  parseThrottle(len, buf);
  parseElevator(len, buf);
  parseAileron(len, buf);
  parseButtons(len, buf);
  parseFireButton(len, buf);
  
  // Check for changes and display them
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_values[i] != previous_channel_values[i]) {
      // Display channel change
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(channel_names[i]);
      Serial.print(" = ");
      Serial.print(channel_values[i]);
      Serial.println("us");
      
      // Update previous value
      previous_channel_values[i] = channel_values[i];
    }
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Joystick Complete Control Mapper ===");
  Serial.println("Designed for Turtle Beach VelocityOne Flightstick");
  Serial.println("Mapping: Throttle, Elevator, Aileron, 8 Buttons, Fire");
  Serial.println("Waiting for joystick connection...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
  }

  // Initialize button states
  for (int i = 0; i < 9; i++) {
    button_states[i] = false;
    previous_button_states[i] = false;
  }

  // Initialize the USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (1);
  }
  Serial.println("USB Host Shield Initialized");
  Serial.println("Connect your Turtle Beach VelocityOne Flightstick...");

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);
  
  Serial.println("\n=== Control Mapper Ready ===");
  Serial.println("Move joystick and press buttons to see channel changes...");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();
  
  // Add a heartbeat to show the system is running
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 10000) { // Every 10 seconds
    Serial.print("System running - Packets received: ");
    Serial.println(data_packet_count);
    last_heartbeat = millis();
  }
}
