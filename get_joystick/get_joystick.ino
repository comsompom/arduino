/*******************************************************************************
 * USB Joystick Setup & Monitor
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
 * Two-mode joystick monitor:
 * 1. SETUP MODE: Remembers joystick movements/buttons to channels
 * 2. LOOP MODE: Shows channel number and value when joystick moves
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 20      // Total number of channels to monitor
#define MAX_DATA_LENGTH 64   // Maximum data packet length
#define SETUP_TIMEOUT 30000  // 30 seconds timeout for setup mode

// Array to hold the values for each channel
unsigned int channel_values[NUM_CHANNELS];
unsigned int previous_channel_values[NUM_CHANNELS];
bool channels_changed = false;

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel mapping storage
struct ChannelMapping {
  uint8_t data_byte;        // Which byte in the data packet
  uint8_t data_length;      // How many bytes this channel uses
  bool is_16bit;            // Whether this is a 16-bit value
  bool is_button;           // Whether this is a button (0/1) or axis (0-255)
  uint8_t button_mask;      // For buttons: which bit to check
  uint8_t button_shift;     // For buttons: how many bits to shift
  bool is_active;           // Whether this mapping is active
  String description;       // Human-readable description
};

ChannelMapping channel_mappings[NUM_CHANNELS];

// System state
enum SystemMode {
  MODE_SETUP,
  MODE_LOOP
};

SystemMode current_mode = MODE_SETUP;
unsigned long setup_start_time = 0;
bool setup_complete = false;
int data_packet_count = 0;

// Data format detection
enum DataFormat {
  FORMAT_UNKNOWN,
  FORMAT_8BIT,
  FORMAT_16BIT
};

DataFormat detected_format = FORMAT_UNKNOWN;

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
  void detectDataFormat(uint8_t len, uint8_t *buf);
  void processSetupMode(uint8_t len, uint8_t *buf);
  void processLoopMode(uint8_t len, uint8_t *buf);
  void autoMapChannels(uint8_t len, uint8_t *buf);
  void displayChannelMappings(); // Moved back to protected
};

// Constructor for our parser
JoystickEvents::JoystickEvents() {}

// This function is called by the USB Host library every time the joystick sends a new data packet.
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  OnJoystickData(len, buf);
}

void JoystickEvents::detectDataFormat(uint8_t len, uint8_t *buf) {
  data_packet_count++;
  
  if (data_packet_count == 1) {
    Serial.print("First data packet received - Length: ");
    Serial.print(len);
    Serial.print(" bytes: ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // Try to detect format based on data length and values
    if (len >= 8) {
      // Check if values look like 8-bit (0-255)
      bool looks_like_8bit = true;
      for (uint8_t i = 0; i < 8; i++) {
        if (buf[i] > 255) {
          looks_like_8bit = false;
          break;
        }
      }
      
      if (looks_like_8bit) {
        detected_format = FORMAT_8BIT;
        Serial.println("Detected: 8-bit format (0-255)");
      } else {
        detected_format = FORMAT_16BIT;
        Serial.println("Detected: 16-bit format");
      }
    } else {
      Serial.println("Warning: Data packet too short for analysis");
    }
  }
}

void JoystickEvents::autoMapChannels(uint8_t len, uint8_t *buf) {
  Serial.println("\n=== Auto-Mapping Channels ===");
  
  // Clear existing mappings
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_mappings[i].is_active = false;
  }
  
  int channel_index = 0;
  
  // Map first 4 bytes as potential axes (8-bit)
  for (int i = 0; i < min(4, len) && channel_index < NUM_CHANNELS; i++) {
    channel_mappings[channel_index].data_byte = i;
    channel_mappings[channel_index].data_length = 1;
    channel_mappings[channel_index].is_16bit = false;
    channel_mappings[channel_index].is_button = false;
    channel_mappings[channel_index].is_active = true;
    channel_mappings[channel_index].description = "Axis " + String(i + 1);
    channel_index++;
  }
  
  // Map remaining bytes as potential buttons
  for (int i = 4; i < len && channel_index < NUM_CHANNELS; i++) {
    channel_mappings[channel_index].data_byte = i;
    channel_mappings[channel_index].data_length = 1;
    channel_mappings[channel_index].is_16bit = false;
    channel_mappings[channel_index].is_button = true;
    channel_mappings[channel_index].button_mask = 0x01;
    channel_mappings[channel_index].button_shift = 0;
    channel_mappings[channel_index].is_active = true;
    channel_mappings[channel_index].description = "Button " + String(i - 3);
    channel_index++;
  }
  
  Serial.print("Auto-mapped ");
  Serial.print(channel_index);
  Serial.println(" channels");
  displayChannelMappings();
}

void JoystickEvents::displayChannelMappings() {
  Serial.println("\n=== Current Channel Mappings ===");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_mappings[i].is_active) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": Byte ");
      Serial.print(channel_mappings[i].data_byte);
      Serial.print(" (");
      Serial.print(channel_mappings[i].description);
      Serial.print(") - ");
      if (channel_mappings[i].is_button) {
        Serial.println("Button");
      } else {
        Serial.println("Axis");
      }
    }
  }
  Serial.println("================================");
}

void JoystickEvents::processSetupMode(uint8_t len, uint8_t *buf) {
  // Auto-map channels on first data packet
  if (data_packet_count == 1) {
    autoMapChannels(len, buf);
  }
  
  // Show raw data for setup
  Serial.print("Setup Mode - Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Process each active channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_mappings[i].is_active) {
      uint8_t byte_index = channel_mappings[i].data_byte;
      if (byte_index < len) {
        if (channel_mappings[i].is_button) {
          // Button value (0 or 1)
          bool button_pressed = (buf[byte_index] & channel_mappings[i].button_mask) != 0;
          channel_values[i] = button_pressed ? 2000 : 1000;
        } else {
          // Axis value (0-255 mapped to 1000-2000)
          uint8_t raw_value = buf[byte_index];
          channel_values[i] = map(raw_value, 0, 255, 1000, 2000);
        }
      }
    }
  }
}

void JoystickEvents::processLoopMode(uint8_t len, uint8_t *buf) {
  // Process each active channel
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_mappings[i].is_active) {
      uint8_t byte_index = channel_mappings[i].data_byte;
      if (byte_index < len) {
        int old_value = channel_values[i];
        
        if (channel_mappings[i].is_button) {
          // Button value (0 or 1)
          bool button_pressed = (buf[byte_index] & channel_mappings[i].button_mask) != 0;
          channel_values[i] = button_pressed ? 2000 : 1000;
        } else {
          // Axis value (0-255 mapped to 1000-2000)
          uint8_t raw_value = buf[byte_index];
          channel_values[i] = map(raw_value, 0, 255, 1000, 2000);
        }
        
        // Check if value changed
        if (channel_values[i] != old_value) {
          // Display channel change
          Serial.print("CH");
          Serial.print(i + 1);
          Serial.print(": ");
          Serial.print(channel_mappings[i].description);
          Serial.print(" = ");
          Serial.print(channel_values[i]);
          Serial.print("us");
          
          if (channel_mappings[i].is_button) {
            Serial.print(" (");
            Serial.print(channel_values[i] == 2000 ? "PRESSED" : "RELEASED");
            Serial.print(")");
          }
          Serial.println();
        }
      }
    }
  }
}

// *** MAIN DATA PROCESSING FUNCTION ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // Detect data format on first packet
  if (data_packet_count == 0) {
    detectDataFormat(len, buf);
  }
  
  // Process based on current mode
  if (current_mode == MODE_SETUP) {
    processSetupMode(len, buf);
  } else {
    processLoopMode(len, buf);
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Joystick Setup & Monitor ===");
  Serial.println("Designed for Turtle Beach VelocityOne Flightstick");
  Serial.println("Starting in SETUP MODE...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
    channel_mappings[i].is_active = false;
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
  
  Serial.println("\n=== SETUP MODE ===");
  Serial.println("Move joystick axes and press buttons to map channels");
  Serial.println("Press 'Y' on keyboard when ready to switch to LOOP MODE");
  Serial.println("Setup will auto-timeout in 30 seconds");
  Serial.println("=============================================");
  
  setup_start_time = millis();
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Check for keyboard input to switch modes
  if (Serial.available() && current_mode == MODE_SETUP) {
    char input = Serial.read();
    if (input == 'Y' || input == 'y') {
      switchToLoopMode();
    }
  }
  
  // Check for setup timeout
  if (current_mode == MODE_SETUP && !setup_complete) {
    if (millis() - setup_start_time > SETUP_TIMEOUT) {
      Serial.println("\nSetup timeout - switching to LOOP MODE automatically");
      switchToLoopMode();
    }
  }
  
  // Add a heartbeat to show the system is running
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 10000) { // Every 10 seconds
    if (current_mode == MODE_SETUP) {
      Serial.print("Setup Mode - Packets received: ");
      Serial.print(data_packet_count);
      Serial.print(" - Time remaining: ");
      Serial.print((SETUP_TIMEOUT - (millis() - setup_start_time)) / 1000);
      Serial.println(" seconds");
    } else {
      Serial.print("Loop Mode - Packets received: ");
      Serial.print(data_packet_count);
      Serial.println(" - Monitoring channels...");
    }
    last_heartbeat = millis();
  }
}

// Global function to display channel mappings
void displayChannelMappingsGlobal() {
  Serial.println("\n=== Current Channel Mappings ===");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_mappings[i].is_active) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": Byte ");
      Serial.print(channel_mappings[i].data_byte);
      Serial.print(" (");
      Serial.print(channel_mappings[i].description);
      Serial.print(") - ");
      if (channel_mappings[i].is_button) {
        Serial.println("Button");
      } else {
        Serial.println("Axis");
      }
    }
  }
  Serial.println("================================");
}

void switchToLoopMode() {
  current_mode = MODE_LOOP;
  setup_complete = true;
  
  Serial.println("\n=== SWITCHING TO LOOP MODE ===");
  Serial.println("Channel mappings:");
  displayChannelMappingsGlobal();
  Serial.println("\nNow monitoring channels - move joystick to see changes");
  Serial.println("=============================================");
}
