/*******************************************************************************
 * USB Joystick Setup & Monitor - MINIMAL MEMORY VERSION
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
 * Minimal memory version with manual setup
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- Configuration --
#define NUM_CHANNELS 4       // Reduced to minimum
#define MAX_DATA_LENGTH 8    // Reduced to minimum

// Array to hold the values for each channel
uint16_t channel_values[NUM_CHANNELS];
uint16_t previous_channel_values[NUM_CHANNELS];

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Minimal channel mapping storage
struct ChannelMapping {
  uint8_t data_byte;
  bool is_button;
  bool is_active;
  char description[4];      // Very short descriptions
};

ChannelMapping channel_mappings[NUM_CHANNELS];

// System state
enum SystemMode {
  MODE_SETUP,
  MODE_LOOP
};

enum SetupStep {
  STEP_ELEVATOR,
  STEP_AILERON,
  STEP_THROTTLE,
  STEP_ARMING,
  STEP_COMPLETE
};

SystemMode current_mode = MODE_SETUP;
SetupStep current_step = STEP_ELEVATOR;
int data_packet_count = 0;

// This class is where we parse the joystick's raw data
class JoystickEvents : public HIDReportParser {
public:
  JoystickEvents();
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);

protected:
  void OnJoystickData(uint8_t len, uint8_t *buf);
  void processSetupMode(uint8_t len, uint8_t *buf);
  void processLoopMode(uint8_t len, uint8_t *buf);
};

// Constructor for our parser
JoystickEvents::JoystickEvents() {}

// This function is called by the USB Host library every time the joystick sends a new data packet.
void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  OnJoystickData(len, buf);
}

void JoystickEvents::processSetupMode(uint8_t len, uint8_t *buf) {
  data_packet_count++;
  
  if (data_packet_count == 1) {
    Serial.print("Data: ");
    Serial.print(len);
    Serial.print(" bytes - ");
    for (uint8_t i = 0; i < min(len, 4); i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
  
  // Show activity for current step
  int step_index = (int)current_step;
  if (step_index < 4 && step_index < len) {
    Serial.print("Step ");
    Serial.print(step_index + 1);
    Serial.print(": Byte ");
    Serial.print(step_index);
    Serial.print(" = ");
    Serial.println(buf[step_index]);
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
          bool button_pressed = (buf[byte_index] & 0x01) != 0;
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
            Serial.print(channel_values[i] == 2000 ? "ON" : "OFF");
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
  Serial.println("MINIMAL MEMORY VERSION");

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

  // Set the HID parser
  Hid.SetReportParser(0, &JoyEvents);
  
  Serial.println("Manual Setup Mode");
  Serial.println("Press '1' to complete each step");
  startSetupStep();
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Check for keyboard input during setup
  if (Serial.available() && current_mode == MODE_SETUP) {
    char input = Serial.read();
    if (input == '1') {
      // User finished current step
      completeSetupStep();
    } else if (input == 'Q' || input == 'q') {
      Serial.println("Force exit - switching to LOOP MODE");
      createDefaultMappings();
      switchToLoopMode();
    }
  }
  
  // Add a heartbeat to show the system is running
  static unsigned long last_heartbeat = 0;
  if (millis() - last_heartbeat > 5000) { // Every 5 seconds
    if (current_mode == MODE_SETUP) {
      Serial.print("Setup: ");
      Serial.print(data_packet_count);
      Serial.print(" packets, Step ");
      Serial.print((int)current_step + 1);
      Serial.println("/4");
      
      // Check USB status
      int state = Usb.getUsbTaskState();
      Serial.print("USB: ");
      switch (state) {
        case USB_STATE_DETACHED: Serial.println("DETACHED"); break;
        case USB_STATE_ADDRESSING: Serial.println("ADDRESSING"); break;
        case USB_STATE_CONFIGURING: Serial.println("CONFIGURING"); break;
        case USB_STATE_RUNNING: Serial.println("RUNNING"); break;
        default: Serial.println("UNKNOWN"); break;
      }
    } else {
      Serial.print("Loop: ");
      Serial.print(data_packet_count);
      Serial.println(" packets");
    }
    last_heartbeat = millis();
  }
}

// Create default channel mappings
void createDefaultMappings() {
  Serial.println("Creating default mappings...");
  
  // Clear existing mappings
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_mappings[i].is_active = false;
  }
  
  // Create default mappings
  for (int i = 0; i < 4; i++) {
    channel_mappings[i].data_byte = i;
    channel_mappings[i].is_button = (i == 3); // Last one is button
    channel_mappings[i].is_active = true;
    if (i == 0) strcpy(channel_mappings[i].description, "Elv");
    else if (i == 1) strcpy(channel_mappings[i].description, "Ail");
    else if (i == 2) strcpy(channel_mappings[i].description, "Thr");
    else strcpy(channel_mappings[i].description, "Arm");
  }
  
  Serial.println("Default: 3 axes, 1 button");
}

// Display channel mappings
void displayChannelMappings() {
  Serial.println("Channel mappings:");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (channel_mappings[i].is_active) {
      Serial.print("CH");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(channel_mappings[i].description);
      Serial.print(" (");
      Serial.print(channel_mappings[i].data_byte);
      Serial.print(") ");
      if (channel_mappings[i].is_button) {
        Serial.println("Btn");
      } else {
        Serial.println("Axis");
      }
    }
  }
}

void startSetupStep() {
  switch (current_step) {
    case STEP_ELEVATOR:
      Serial.println("=== STEP 1: ELEVATOR ===");
      Serial.println("Move ELEVATOR control");
      Serial.println("Press '1' when done");
      break;
    case STEP_AILERON:
      Serial.println("=== STEP 2: AILERON ===");
      Serial.println("Move AILERON control");
      Serial.println("Press '1' when done");
      break;
    case STEP_THROTTLE:
      Serial.println("=== STEP 3: THROTTLE ===");
      Serial.println("Move THROTTLE control");
      Serial.println("Press '1' when done");
      break;
    case STEP_ARMING:
      Serial.println("=== STEP 4: ARMING ===");
      Serial.println("Press ARMING button");
      Serial.println("Press '1' when done");
      break;
    default:
      break;
  }
}

void completeSetupStep() {
  int step_index = (int)current_step;
  
  if (step_index < 4) {
    // Create mapping for this step
    channel_mappings[step_index].data_byte = step_index;
    channel_mappings[step_index].is_active = true;
    channel_mappings[step_index].is_button = (step_index == 3);
    
    switch (current_step) {
      case STEP_ELEVATOR:
        strcpy(channel_mappings[step_index].description, "Elv");
        Serial.println("Elevator mapped to CH1");
        break;
      case STEP_AILERON:
        strcpy(channel_mappings[step_index].description, "Ail");
        Serial.println("Aileron mapped to CH2");
        break;
      case STEP_THROTTLE:
        strcpy(channel_mappings[step_index].description, "Thr");
        Serial.println("Throttle mapped to CH3");
        break;
      case STEP_ARMING:
        strcpy(channel_mappings[step_index].description, "Arm");
        Serial.println("Arming mapped to CH4");
        break;
    }
    
    // Move to next step
    current_step = (SetupStep)(step_index + 1);
    
    if (current_step == STEP_COMPLETE) {
      Serial.println("=== SETUP COMPLETE ===");
      switchToLoopMode();
    } else {
      startSetupStep();
    }
  }
}

void switchToLoopMode() {
  current_mode = MODE_LOOP;
  
  Serial.println("Switching to LOOP MODE");
  displayChannelMappings();
  Serial.println("Monitoring channels - move joystick");
}
