/*******************************************************************************
 * USB Joystick Channel Monitor
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Reads USB HID data from a joystick and displays the current channel values
 * in real-time via Serial Monitor. This is useful for debugging and mapping
 * joystick axes to specific channels.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 20      // Total number of channels to monitor
#define DISPLAY_INTERVAL 100 // How often to display values (milliseconds)

// Array to hold the values for each channel
unsigned int channel_values[NUM_CHANNELS];
unsigned long last_display_time = 0;

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
}

// *** THIS IS THE MOST IMPORTANT FUNCTION TO CUSTOMIZE ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // This is where you translate the raw joystick data buffer (buf)
  // into your channel_values array.
  
  // --- STEP 1: DEBUGGING - Find your axis values ---
  // Uncomment the following block to print the raw data from the joystick.
  // Open the Serial Monitor at 115200 baud, move one axis at a time,
  // and observe which bytes in the output change.
  
  Serial.print("Raw HID Data: ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  

  // --- STEP 2: MAPPING - Update this section based on your findings ---
  // The following is an EXAMPLE based on a typical joystick.
  // Your specific joystick might be different!
  // Joysticks often use 10-bit values (0-1023) or 16-bit values.
  // Let's assume 10-bit values split across two bytes.

  // Example: Aileron (X-Axis) might be in bytes 0 and 1
  int aileron_raw = (buf[1] << 8) | buf[0]; // Combine two bytes for a 16-bit value
  channel_values[0] = map(aileron_raw, 0, 1023, 1000, 2000); // CH1: Aileron

  // Example: Elevator (Y-Axis) might be in bytes 2 and 3
  int elevator_raw = (buf[3] << 8) | buf[2];
  channel_values[1] = map(elevator_raw, 0, 1023, 1000, 2000); // CH2: Elevator
  // channel_values[1] = map(elevator_raw, 1023, 0, 1000, 2000); // Use this if axis is reversed

  // Example: Throttle might be in bytes 4 and 5
  int throttle_raw = (buf[5] << 8) | buf[4];
  channel_values[2] = map(throttle_raw, 0, 1023, 1000, 2000); // CH3: Throttle
  
  // Example: Rudder (Twist) might be in bytes 6 and 7
  int rudder_raw = (buf[7] << 8) | buf[6];
  channel_values[3] = map(rudder_raw, 0, 1023, 1000, 2000); // CH4: Rudder

  // Example: Additional buttons/axes (if available)
  // You can add more mappings here based on your joystick's capabilities
  
  // Make sure values are within the valid range
  for(int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = constrain(channel_values[i], 1000, 2000);
  }
}

// Create an instance of our parser class
JoystickEvents JoyEvents;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Joystick Channel Monitor ===");
  Serial.println("Waiting for joystick connection...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500; // Center position
  }

  // Initialize the USB Host Shield
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    while (1); // Halt
  }
  Serial.println("USB Host Shield Initialized");
  Serial.println("Connect your joystick...");

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);
  
  Serial.println("\n=== Channel Values ===");
  Serial.println("Format: Channel Name: Value (us) | Visual Bar");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Display channel values at regular intervals
  unsigned long current_time = millis();
  if (current_time - last_display_time >= DISPLAY_INTERVAL) {
    displayChannelValues();
    last_display_time = current_time;
  }
}

void displayChannelValues() {
  // Clear the display area (works best with Serial Monitor set to "No line ending")
  Serial.print("\033[2J\033[H"); // Clear screen and move cursor to top
  
  Serial.println("=== USB Joystick Channel Monitor ===");
  Serial.println("Raw data is shown above, channel values below:");
  Serial.println("=============================================");
  
  // Display each channel value with a visual bar
  for (int i = 0; i < NUM_CHANNELS; i++) {
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
    
    // Display channel info
    Serial.print(channel_names[i]);
    Serial.print(": ");
    Serial.print(channel_values[i]);
    Serial.print("us | ");
    Serial.print(percentage);
    Serial.print("% | ");
    Serial.println(bar);
  }
  
  Serial.println("=============================================");
  Serial.println("Center: 1500us | Min: 1000us | Max: 2000us");
  Serial.println("Move joystick axes to see changes...");
}

// Alternative display function for simpler output
void displaySimpleValues() {
  Serial.println("--- Channel Values ---");
  for (int i = 0; i < 8; i++) { // Show first 8 channels
    Serial.print(channel_names[i]);
    Serial.print(": ");
    Serial.print(channel_values[i]);
    Serial.print("us");
    if (i % 2 == 1) {
      Serial.println(); // New line every 2 channels
    } else {
      Serial.print("\t\t");
    }
  }
  Serial.println();
}
