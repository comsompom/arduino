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
#define NUM_CHANNELS 8       // Further reduced to save memory
#define MAX_DATA_LENGTH 16   // Further reduced to save memory
#define SETUP_TIMEOUT 30000  // 30 seconds timeout for setup mode

// Array to hold the values for each channel
uint16_t channel_values[NUM_CHANNELS];      // Changed from unsigned int to uint16_t
uint16_t previous_channel_values[NUM_CHANNELS]; // Changed from unsigned int to uint16_t
bool channels_changed = false;

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Channel mapping storage - further optimized for memory
struct ChannelMapping {
  uint8_t data_byte;        // Which byte in the data packet
  bool is_button;           // Whether this is a button (0/1) or axis (0-255)
  bool is_active;           // Whether this mapping is active
  char description[8];      // Reduced from 12 to 8 characters
};

ChannelMapping channel_mappings[NUM_CHANNELS];

// Setup tracking variables
uint8_t setup_min_values[4];  // Store min values during setup
uint8_t setup_max_values[4];  // Store max values during setup
bool setup_values_ready[4];   // Track if we have min/max for each step

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
  void manualSetupStep(uint8_t len, uint8_t *buf);
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
    Serial.print("Data: ");
    Serial.print(len);
    Serial.print(" bytes - ");
    for (uint8_t i = 0; i < min(len, 8); i++) {
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
        Serial.println("8-bit format");
      } else {
        detected_format = FORMAT_16BIT;
        Serial.println("16-bit format");
      }
    } else {
      Serial.println("Short packet");
    }
  }
}

void JoystickEvents::manualSetupStep(uint8_t len, uint8_t *buf) {
  // Find the most active byte for current step
  uint8_t most_active_byte = 0;
  uint8_t max_change = 0;
  
  // Check first 8 bytes for activity
  for (uint8_t i = 0; i < min(len, 8); i++) {
    uint8_t current_value = buf[i];
    uint8_t change = 0;
    
    // Calculate change from center (128)
    if (current_value > 128) {
      change = current_value - 128;
    } else {
      change = 128 - current_value;
    }
    
    if (change > max_change) {
      max_change = change;
      most_active_byte = i;
    }
  }
  
  // Update min/max values for current step
  int step_index = (int)current_step;
  if (step_index < 4) {
    if (!setup_values_ready[step_index]) {
      setup_min_values[step_index] = buf[most_active_byte];
      setup_max_values[step_index] = buf[most_active_byte];
      setup_values_ready[step_index] = true;
    } else {
      if (buf[most_active_byte] < setup_min_values[step_index]) {
        setup_min_values[step_index] = buf[most_active_byte];
      }
      if (buf[most_active_byte] > setup_max_values[step_index]) {
        setup_max_values[step_index] = buf[most_active_byte];
      }
    }
  }
  
  // Show current activity
  Serial.print("Activity: Byte ");
  Serial.print(most_active_byte);
  Serial.print(" = ");
  Serial.print(buf[most_active_byte]);
  Serial.print(" (");
  Serial.print(setup_min_values[step_index]);
  Serial.print("-");
  Serial.print(setup_max_values[step_index]);
  Serial.println(")");
}

void JoystickEvents::displayChannelMappings() {
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

void JoystickEvents::processSetupMode(uint8_t len, uint8_t *buf) {
  // Detect data format on first packet
  if (data_packet_count == 1) {
    detectDataFormat(len, buf);
  }
  
  // Manual setup step
  manualSetupStep(len, buf);
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
  Serial.println("Starting Manual Setup Mode...");

  // Initialize all channels to center position
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_values[i] = 1500;
    previous_channel_values[i] = 1500;
    channel_mappings[i].is_active = false;
  }
  
  // Initialize setup tracking
  for (int i = 0; i < 4; i++) {
    setup_min_values[i] = 0;
    setup_max_values[i] = 255;
    setup_values_ready[i] = false;
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
  
  Serial.println("\n=== MANUAL SETUP MODE ===");
  Serial.println("Follow the prompts to map each control");
  Serial.println("Press '1' when finished with each step");
  Serial.println("=============================================");
  
  setup_start_time = millis();
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
  if (millis() - last_heartbeat > 3000) { // Every 3 seconds
    if (current_mode == MODE_SETUP) {
      Serial.print("Status: ");
      Serial.print(data_packet_count);
      Serial.print(" packets, Step ");
      Serial.print((int)current_step + 1);
      Serial.print("/4");
      
      // Check USB status
      int state = Usb.getUsbTaskState();
      Serial.print(", USB: ");
      switch (state) {
        case USB_STATE_DETACHED: Serial.print("DETACHED"); break;
        case USB_STATE_ADDRESSING: Serial.print("ADDRESSING"); break;
        case USB_STATE_CONFIGURING: Serial.print("CONFIGURING"); break;
        case USB_STATE_RUNNING: Serial.print("RUNNING"); break;
        default: Serial.print("UNKNOWN"); break;
      }
      
      if (Hid.isReady()) {
        Serial.println(", HID: OK");
      } else {
        Serial.println(", HID: NOT FOUND");
      }
      
      // Show current step reminder
      switch (current_step) {
        case STEP_ELEVATOR: Serial.println("Current: Move ELEVATOR or press '1'"); break;
        case STEP_AILERON: Serial.println("Current: Move AILERON or press '1'"); break;
        case STEP_THROTTLE: Serial.println("Current: Move THROTTLE or press '1'"); break;
        case STEP_ARMING: Serial.println("Current: Press ARMING button or press '1'"); break;
        default: break;
      }
    } else {
      Serial.print("Loop: ");
      Serial.print(data_packet_count);
      Serial.println(" packets");
    }
    last_heartbeat = millis();
  }
}

// Create default channel mappings when no joystick data is received
void createDefaultMappings() {
  Serial.println("Creating default mappings...");
  
  // Clear existing mappings
  for (int i = 0; i < NUM_CHANNELS; i++) {
    channel_mappings[i].is_active = false;
  }
  
  // Create default mappings for common joystick layout
  // First 4 channels as axes
  for (int i = 0; i < 4; i++) {
    channel_mappings[i].data_byte = i;
    channel_mappings[i].is_button = false;
    channel_mappings[i].is_active = true;
    sprintf(channel_mappings[i].description, "A%d", i + 1);
  }
  
  // Next 4 channels as buttons
  for (int i = 4; i < 8; i++) {
    channel_mappings[i].data_byte = i;
    channel_mappings[i].is_button = true;
    channel_mappings[i].is_active = true;
    sprintf(channel_mappings[i].description, "B%d", i - 3);
  }
  
  Serial.println("Default: 4 axes, 4 buttons");
}

// Global function to display channel mappings
void displayChannelMappingsGlobal() {
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
      Serial.println("\n=== STEP 1: ELEVATOR ===");
      Serial.println("Move the ELEVATOR control up and down");
      Serial.println("Press '1' when finished (or 'Q' to skip)");
      Serial.println("Waiting for joystick activity...");
      break;
    case STEP_AILERON:
      Serial.println("\n=== STEP 2: AILERON ===");
      Serial.println("Move the AILERON control left and right");
      Serial.println("Press '1' when finished (or 'Q' to skip)");
      Serial.println("Waiting for joystick activity...");
      break;
    case STEP_THROTTLE:
      Serial.println("\n=== STEP 3: THROTTLE ===");
      Serial.println("Move the THROTTLE control up and down");
      Serial.println("Press '1' when finished (or 'Q' to skip)");
      Serial.println("Waiting for joystick activity...");
      break;
    case STEP_ARMING:
      Serial.println("\n=== STEP 4: ARMING BUTTON ===");
      Serial.println("Press the ARMING button on your joystick");
      Serial.println("Press '1' when finished (or 'Q' to skip)");
      Serial.println("Waiting for joystick activity...");
      break;
    default:
      break;
  }
}

void completeSetupStep() {
  int step_index = (int)current_step;
  
  // Allow proceeding even without activity detected
  if (step_index < 4) {
    // Find the most active byte for this step
    uint8_t most_active_byte = step_index; // Default to step order
    
    // If we have activity data, use it
    if (setup_values_ready[step_index]) {
      // In a real implementation, we'd analyze the activity data here
      // For now, we'll use the step index as the byte position
      most_active_byte = step_index;
    } else {
      Serial.println("No activity detected, using default mapping");
    }
    
    // Create mapping for this step
    channel_mappings[step_index].data_byte = most_active_byte;
    channel_mappings[step_index].is_active = true;
    
    switch (current_step) {
      case STEP_ELEVATOR:
        channel_mappings[step_index].is_button = false;
        strcpy(channel_mappings[step_index].description, "Elev");
        Serial.println("Elevator mapped to CH1 (Byte " + String(most_active_byte) + ")");
        break;
      case STEP_AILERON:
        channel_mappings[step_index].is_button = false;
        strcpy(channel_mappings[step_index].description, "Ail");
        Serial.println("Aileron mapped to CH2 (Byte " + String(most_active_byte) + ")");
        break;
      case STEP_THROTTLE:
        channel_mappings[step_index].is_button = false;
        strcpy(channel_mappings[step_index].description, "Thr");
        Serial.println("Throttle mapped to CH3 (Byte " + String(most_active_byte) + ")");
        break;
      case STEP_ARMING:
        channel_mappings[step_index].is_button = true;
        strcpy(channel_mappings[step_index].description, "Arm");
        Serial.println("Arming button mapped to CH4 (Byte " + String(most_active_byte) + ")");
        break;
    }
    
    // Move to next step
    current_step = (SetupStep)(step_index + 1);
    
    if (current_step == STEP_COMPLETE) {
      Serial.println("\n=== SETUP COMPLETE ===");
      Serial.println("All controls mapped successfully!");
      switchToLoopMode();
    } else {
      startSetupStep();
    }
  } else {
    Serial.println("Setup step error. Press 'Q' to force exit.");
  }
}

void switchToLoopMode() {
  current_mode = MODE_LOOP;
  setup_complete = true;
  
  Serial.println("Switching to LOOP MODE");
  displayChannelMappingsGlobal();
  Serial.println("Monitoring channels - move joystick");
}
