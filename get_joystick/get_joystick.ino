/*******************************************************************************
 * USB Joystick Debug - Turtle Beach VelocityOne Flightstick
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
 * Comprehensive debugging sketch to identify USB connection issues
 * and get the Turtle Beach VelocityOne Flightstick working.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>
#include <hidboot.h>

// -- Configuration --
#define NUM_CHANNELS 20
#define DEBUG_MODE 1

// Array to hold the values for each channel
unsigned int channel_values[NUM_CHANNELS];
unsigned int previous_channel_values[NUM_CHANNELS];
bool channels_changed = false;

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Device detection
bool device_connected = false;
bool hid_device_found = false;
unsigned long last_device_check = 0;

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

// *** MAIN DATA PROCESSING FUNCTION ***
void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
  // Print raw data for debugging
  Serial.print("HID Data (len=");
  Serial.print(len);
  Serial.print("): ");
  for (uint8_t i = 0; i < len; i++) {
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Try to map the data
  if (len >= 4) {
    // Try different byte combinations for the axes
    int aileron_raw = buf[0];
    int elevator_raw = buf[1];
    int throttle_raw = buf[2];
    int rudder_raw = buf[3];
    
    // Map the raw values to channel range (1000-2000us)
    channel_values[0] = map(aileron_raw, 0, 255, 1000, 2000);
    channel_values[1] = map(elevator_raw, 0, 255, 1000, 2000);
    channel_values[2] = map(throttle_raw, 0, 255, 1000, 2000);
    channel_values[3] = map(rudder_raw, 0, 255, 1000, 2000);
    
    // Debug: Print raw values
    Serial.print("Raw values - Aileron:");
    Serial.print(aileron_raw);
    Serial.print(" Elevator:");
    Serial.print(elevator_raw);
    Serial.print(" Throttle:");
    Serial.print(throttle_raw);
    Serial.print(" Rudder:");
    Serial.println(rudder_raw);
  }
  
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
  Serial.println("=== USB Joystick Debug - Turtle Beach VelocityOne ===");
  Serial.println("Comprehensive debugging for USB connection issues");
  Serial.println("Waiting for joystick connection...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
  }

  // Initialize the USB Host Shield
  Serial.println("Initializing USB Host Shield...");
  if (Usb.Init() == -1) {
    Serial.println("USB Host Shield initialization failed!");
    Serial.println("Check your wiring:");
    Serial.println("- INT pin (usually pin 7)");
    Serial.println("- SS pin (usually pin 10)");
    Serial.println("- MOSI, MISO, SCK pins");
    Serial.println("- Power and ground connections");
    while (1);
  }
  Serial.println("USB Host Shield Initialized Successfully");

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);
  
  Serial.println("\n=== Debug Monitor Ready ===");
  Serial.println("System will check for USB devices every 2 seconds");
  Serial.println("Move joystick axes to see changes...");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Check for device connection every 2 seconds
  if (millis() - last_device_check > 2000) {
    checkDeviceStatus();
    last_device_check = millis();
  }

  // Only display if channels have changed
  if (channels_changed) {
    displayChannelValues();
    channels_changed = false;
    
    // Update previous values
    for (int i = 0; i < NUM_CHANNELS; i++) {
      previous_channel_values[i] = channel_values[i];
    }
  }
}

void checkDeviceStatus() {
  Serial.println("\n=== Device Status Check ===");
  
  // Check if any USB device is connected
  if (Usb.getUsbTaskState() == USB_STATE_RUNNING) {
    Serial.println("USB Host Shield: RUNNING");
    
    // Check if HID device is found
    if (Hid.isReady()) {
      Serial.println("HID Device: CONNECTED");
      hid_device_found = true;
      
      // Get device information
      Serial.print("Device VID: 0x");
      Serial.println(Hid.getVID(), HEX);
      Serial.print("Device PID: 0x");
      Serial.println(Hid.getPID(), HEX);
      
    } else {
      Serial.println("HID Device: NOT FOUND");
      hid_device_found = false;
      
      // Troubleshooting tips
      Serial.println("Troubleshooting:");
      Serial.println("1. Make sure joystick is powered on");
      Serial.println("2. Try unplugging and reconnecting the joystick");
      Serial.println("3. Check if joystick works on a computer");
      Serial.println("4. Try a different USB cable");
      Serial.println("5. Check USB Host Shield power supply");
    }
  } else {
    Serial.println("USB Host Shield: NOT RUNNING");
    Serial.print("USB State: ");
    Serial.println(Usb.getUsbTaskState());
  }
  
  Serial.println("================================");
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
