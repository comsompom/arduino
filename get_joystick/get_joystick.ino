/*******************************************************************************
 * USB Joystick Channel Monitor - Advanced Version
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
 * Advanced version that tries multiple data formats to find the correct
 * mapping for the Turtle Beach VelocityOne Flightstick.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 20      // Total number of channels to monitor
#define DEBUG_MODE 1         // Set to 1 for detailed debugging

// Array to hold the values for each channel
unsigned int channel_values[NUM_CHANNELS];
unsigned int previous_channel_values[NUM_CHANNELS];
bool channels_changed = false;

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel names for display
const char* channel_names[NUM_CHANNELS] = {
  "CH1 (Aileron)", "CH2 (Elevator)", "CH3 (Throttle)", "CH4 (Rudder)",
  "CH5", "CH6", "CH7", "CH8", "CH9", "CH10",
  "CH11", "CH12", "CH13", "CH14", "CH15", "CH16",
  "CH17", "CH18", "CH19", "CH20"
};

// Data format detection
enum DataFormat {
  FORMAT_UNKNOWN,
  FORMAT_8BIT,
  FORMAT_16BIT,
  FORMAT_10BIT,
  FORMAT_12BIT
};

DataFormat detected_format = FORMAT_UNKNOWN;
int data_packet_count = 0;

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
  void tryDifferentFormats(uint8_t len, uint8_t *buf);
  void detectDataFormat(uint8_t len, uint8_t *buf);
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

void JoystickEvents::tryDifferentFormats(uint8_t len, uint8_t *buf) {
  if (len < 4) return; // Need at least 4 bytes for basic axes
  
  // Try multiple format combinations
  int aileron_raw, elevator_raw, throttle_raw, rudder_raw;
  
  // Format 1: Direct 8-bit values
  aileron_raw = buf[0];
  elevator_raw = buf[1];
  throttle_raw = buf[2];
  rudder_raw = buf[3];
  
  // Format 2: 16-bit values (little endian)
  int aileron_16bit = (buf[1] << 8) | buf[0];
  int elevator_16bit = (buf[3] << 8) | buf[2];
  int throttle_16bit = (buf[5] << 8) | buf[4];
  int rudder_16bit = (buf[7] << 8) | buf[6];
  
  // Format 3: Alternative byte positions
  int aileron_alt = buf[1];
  int elevator_alt = buf[2];
  int throttle_alt = buf[3];
  int rudder_alt = buf[4];
  
  // Map based on detected format
  switch (detected_format) {
    case FORMAT_8BIT:
      channel_values[0] = map(aileron_raw, 0, 255, 1000, 2000);
      channel_values[1] = map(elevator_raw, 0, 255, 1000, 2000);
      channel_values[2] = map(throttle_raw, 0, 255, 1000, 2000);
      channel_values[3] = map(rudder_raw, 0, 255, 1000, 2000);
      break;
      
    case FORMAT_16BIT:
      channel_values[0] = map(aileron_16bit, 0, 65535, 1000, 2000);
      channel_values[1] = map(elevator_16bit, 0, 65535, 1000, 2000);
      channel_values[2] = map(throttle_16bit, 0, 65535, 1000, 2000);
      channel_values[3] = map(rudder_16bit, 0, 65535, 1000, 2000);
      break;
      
    default:
      // Try 8-bit first, then fallback
      channel_values[0] = map(aileron_raw, 0, 255, 1000, 2000);
      channel_values[1] = map(elevator_raw, 0, 255, 1000, 2000);
      channel_values[2] = map(throttle_raw, 0, 255, 1000, 2000);
      channel_values[3] = map(rudder_raw, 0, 255, 1000, 2000);
      break;
  }
  
  // Debug output
  if (DEBUG_MODE && data_packet_count <= 10) {
    Serial.print("Format ");
    Serial.print(detected_format);
    Serial.print(" - Raw: ");
    Serial.print(aileron_raw);
    Serial.print(",");
    Serial.print(elevator_raw);
    Serial.print(",");
    Serial.print(throttle_raw);
    Serial.print(",");
    Serial.print(rudder_raw);
    Serial.print(" | Mapped: ");
    Serial.print(channel_values[0]);
    Serial.print(",");
    Serial.print(channel_values[1]);
    Serial.print(",");
    Serial.print(channel_values[2]);
    Serial.print(",");
    Serial.println(channel_values[3]);
  }
}

// *** MAIN DATA PROCESSING FUNCTION ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // Detect data format on first packet
  if (data_packet_count == 0) {
    detectDataFormat(len, buf);
  }
  
  // Print raw data for debugging
  if (DEBUG_MODE) {
    Serial.print("HID Data (len=");
    Serial.print(len);
    Serial.print("): ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Try different data formats
  tryDifferentFormats(len, buf);
  
  // Make sure values are within the valid range
  for(int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = constrain(channel_values[i], 1000, 2000);
    
    // Check if this channel value changed
    if (channel_values[i] != previous_channel_values[i]) {
      channels_changed = true;
    }
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Joystick Channel Monitor - Advanced ===");
  Serial.println("Designed for Turtle Beach VelocityOne Flightstick");
  Serial.println("Waiting for joystick connection...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
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
  
  Serial.println("\n=== Advanced Monitor Ready ===");
  Serial.println("System will auto-detect data format");
  Serial.println("Move joystick axes to see changes...");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Only display if channels have changed
  if (channels_changed) {
    displayChannelValues();
    channels_changed = false;
    
    // Update previous values
    for (int i = 0; i < NUM_CHANNELS; i++) {
      previous_channel_values[i] = channel_values[i];
    }
  }
  
  // Add a heartbeat to show the system is running
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 10000) { // Every 10 seconds
    Serial.print("System running - Packets received: ");
    Serial.print(data_packet_count);
    Serial.print(" - Format: ");
    Serial.println(detected_format);
    last_heartbeat = millis();
  }
}

void displayChannelValues() {
  Serial.println("\n=== Channel Values Changed ===");
  Serial.println("Timestamp: " + String(millis()) + "ms");
  Serial.println("=============================================");
  
  // Display only channels that changed
  bool any_changed = false;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_values[i] != previous_channel_values[i]) {
      any_changed = true;
      
      // Calculate percentage for visual bar
      int percentage = map(channel_values[i], 1000, 2000, 0, 100);
      
      // Create visual bar
      char bar[21];
      int bar_length = map(percentage, 0, 100, 0, 20);
      for (int j = 0; j < 20; j++) {
        if (j < bar_length) {
          bar[j] = '#';
        } else {
          bar[j] = '-';
        }
      }
      bar[20] = '\0';
      
      // Display channel info with change indicator
      Serial.print(channel_names[i]);
      Serial.print(": ");
      Serial.print(previous_channel_values[i]);
      Serial.print("us -> ");
      Serial.print(channel_values[i]);
      Serial.print("us | ");
      Serial.print(percentage);
      Serial.print("% | ");
      Serial.println(bar);
    }
  }
  
  if (!any_changed) {
    Serial.println("No channels changed");
  }
  
  Serial.println("=============================================");
}
