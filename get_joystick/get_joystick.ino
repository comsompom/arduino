/*******************************************************************************
 * USB Joystick Raw Data Monitor
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
 * Shows raw HID data from joystick and maps buttons/throttle based on
 * specific data patterns from Turtle Beach VelocityOne Flightstick
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 8       // Reduced for memory efficiency
#define DEBUG_MODE 1         // Set to 1 for detailed debugging

// Array to hold the values for each channel
uint16_t channel_values[NUM_CHANNELS];
uint16_t previous_channel_values[NUM_CHANNELS];
bool channels_changed = false;

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel names for display
const char* channel_names[NUM_CHANNELS] = {
  "CH1 (Throttle)", "CH2 (Button1)", "CH3 (Button2)", "CH4 (Button3)",
  "CH5 (Button4)", "CH6 (Button5)", "CH7 (Button6)", "CH8 (Button7)"
};

// Button states tracking
bool button_states[8] = {false, false, false, false, false, false, false, false};
bool previous_button_states[8] = {false, false, false, false, false, false, false, false};

// Throttle tracking
uint16_t throttle_raw = 0;
uint16_t previous_throttle_raw = 0;

int data_packet_count = 0;

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
  void parseButtons(uint8_t len, uint8_t *buf);
  void parseThrottle(uint8_t len, uint8_t *buf);
};

// Constructor for our parser
JoystickEvents::JoystickEvents() {}

// This function is called by the USB Host library every time the joystick sends a new data packet.
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  OnJoystickData(len, buf);
}

void JoystickEvents::parseButtons(uint8_t len, uint8_t *buf) {
  // Based on the data pattern provided:
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
    
    // Update channel values for buttons (CH2-CH8)
    for (int i = 0; i < 7; i++) {
      channel_values[i + 1] = button_states[i] ? 2000 : 1000;
    }
  }
}

void JoystickEvents::parseThrottle(uint8_t len, uint8_t *buf) {
  // Based on the data pattern provided:
  // Throttle data is in bytes 8-9 (16-bit value)
  // Throttle at zero:     bytes 8-9 = 0xE0 0x80
  // Throttle start:       bytes 8-9 = 0xE0 0x80 (same as zero)
  // Throttle at 50%:      bytes 8-9 = 0xC0 0x73
  // Throttle 100%:        bytes 8-9 = 0xFF 0xFF
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
      
      if (DEBUG_MODE) {
        Serial.print("Throttle: Raw=0x");
        Serial.print(raw_throttle, HEX);
        Serial.print(" Mapped=");
        Serial.print(mapped_throttle);
        Serial.println("us");
      }
    }
  }
}

// *** MAIN DATA PROCESSING FUNCTION ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  data_packet_count++;
  
  // Always print raw HID data
  Serial.print("HID Data (len=");
  Serial.print(len);
  Serial.print("): ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Parse buttons and throttle
  parseButtons(len, buf);
  parseThrottle(len, buf);
  
  // Check for changes
  channels_changed = false;
  
  // Check throttle changes
  if (channel_values[0] != previous_channel_values[0]) {
    channels_changed = true;
  }
  
  // Check button changes
  for (int i = 0; i < 7; i++) {
    if (button_states[i] != previous_button_states[i]) {
      channels_changed = true;
      break;
    }
  }
  
  // Update previous values
  for (int i = 0; i < NUM_CHANNELS; i++) {
    previous_channel_values[i] = channel_values[i];
  }
  
  for (int i = 0; i < 8; i++) {
    previous_button_states[i] = button_states[i];
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Joystick Raw Data Monitor ===");
  Serial.println("Designed for Turtle Beach VelocityOne Flightstick");
  Serial.println("Shows raw HID data and maps buttons/throttle");
  Serial.println("Waiting for joystick connection...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
  }

  // Initialize button states
  for (int i = 0; i < 8; i++) {
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
  
  Serial.println("\n=== Raw Data Monitor Ready ===");
  Serial.println("System will show all HID data packets");
  Serial.println("Move joystick and press buttons to see changes...");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Display channel changes when they occur
  if (channels_changed) {
    displayChannelValues();
    channels_changed = false;
  }
  
  // Add a heartbeat to show the system is running
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 10000) { // Every 10 seconds
    Serial.print("System running - Packets received: ");
    Serial.println(data_packet_count);
    last_heartbeat = millis();
  }
}

void displayChannelValues() {
  Serial.println("\n=== Channel Values Changed ===");
  Serial.println("Timestamp: " + String(millis()) + "ms");
  Serial.println("=============================================");
  
  // Display throttle changes
  if (channel_values[0] != previous_channel_values[0]) {
    Serial.print(channel_names[0]);
    Serial.print(": ");
    Serial.print(previous_channel_values[0]);
    Serial.print("us -> ");
    Serial.print(channel_values[0]);
    Serial.println("us");
  }
  
  // Display button changes
  for (int i = 0; i < 7; i++) {
    if (button_states[i] != previous_button_states[i]) {
      Serial.print(channel_names[i + 1]);
      Serial.print(": ");
      Serial.print(button_states[i] ? "PRESSED" : "RELEASED");
      Serial.print(" (");
      Serial.print(channel_values[i + 1]);
      Serial.println("us)");
    }
  }
  
  Serial.println("=============================================");
}
