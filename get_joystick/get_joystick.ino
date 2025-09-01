/*******************************************************************************
 * USB Joystick Complete Control Mapper - FIXED
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
 * channel changes with channel number and name. This version is corrected
 * based on the provided HID data log.
 *
 * CORRECTIONS MADE:
 * - Aileron mapped to bytes 1 (LSB) & 2 (MSB).
 * - Elevator mapped to bytes 3 (LSB) & 4 (MSB).
 * - Throttle range check removed for correct mapping from 0x0000-0xFFFF.
 * - Buttons B1-B8 mapped to the correct byte 17.
 * - Fire Button mapped to the correct byte 18.
 *
 ******************************************************************************/

 #include <hiduniversal.h>
 #include <usbhub.h>
 
 // -- Configuration --
 #define NUM_CHANNELS 12      // Total channels for all controls
 
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
 
 // This class is where we parse the joystick's raw data
 class JoystickEvents : public HIDReportParser {
 public:
   JoystickEvents();
   void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
 
 protected:
   // Renamed the main parsing function for clarity
   void OnJoystickData(uint8_t len, uint8_t *buf); 
 };
 
 // Constructor for our parser
 JoystickEvents::JoystickEvents() {}
 
 // This function is called by the USB Host library every time the joystick sends a new data packet.
 void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
   // We only care about data packets with a length of 51, as per the log
   if (len == 51) {
     OnJoystickData(len, buf);
   }
 }
 
 // *** MAIN DATA PROCESSING FUNCTION ***
 void JoystickEvents::OnJoystickData(uint8_t len, uint8_t *buf) {
   // --- AXES ---
   // Based on the HID data pattern analysis:
   // Aileron (X-axis): bytes 1 & 2 (16-bit)
   // Elevator (Y-axis): bytes 3 & 4 (16-bit)
   // Throttle: bytes 8 & 9 (16-bit)
 
   // AILERON (Corrected: bytes 1 & 2)
   uint16_t raw_aileron = (buf[2] << 8) | buf[1];
   channel_values[2] = map(raw_aileron, 0x0000, 0xFFFF, 1000, 2000);
 
   // ELEVATOR (Corrected: bytes 3 & 4)
   uint16_t raw_elevator = (buf[4] << 8) | buf[3];
   // Note: Depending on joystick direction, you might need to invert the mapping
   // e.g., map(raw_elevator, 0x0000, 0xFFFF, 2000, 1000);
   channel_values[1] = map(raw_elevator, 0x0000, 0xFFFF, 1000, 2000);
 
                               // THROTTLE (Corrected: Based on actual HID data analysis)
      // Throttle data is in bytes 10-11 (16-bit value)
      // Throttle start (0%):   bytes 10-11 = 0x0000
      // Throttle 25%:          bytes 10-11 = 0x43C0
      // Throttle 50%:          bytes 10-11 = 0x73C0
      // Throttle 75%:          bytes 10-11 = 0xD840
      // Throttle 100%:         bytes 10-11 = 0xFFFF
      uint16_t raw_throttle = (buf[11] << 8) | buf[10];
      
      // Map throttle to channel value (1000-2000us)
      // 0x0000 (0%) -> 1000us
      // 0xFFFF (100%) -> 2000us
      channel_values[0] = map(raw_throttle, 0x0000, 0xFFFF, 1000, 2000);
 
 
       // --- BUTTONS ---
    // Based on the HID data pattern analysis:
    // Buttons B1-B8 are a bitmask in byte 18.
    // Fire button is a bitmask in byte 20.

    // BUTTONS B1-B8 (Corrected: byte 18)
    uint8_t button_byte = buf[18];
    for (int i = 0; i < 8; i++) {
      // Check if the i-th bit is set
      bool button_state = (button_byte & (1 << i)) != 0;
      // Map boolean state to 1000 (released) or 2000 (pressed)
      // Channel mapping: B1->CH4, B2->CH5, ... B8->CH11
      channel_values[i + 3] = button_state ? 2000 : 1000;
    }
    
    // FIRE BUTTON (Corrected: byte 20, bit 1)
    bool fire_pressed = (buf[20] & 0x02) != 0;
    channel_values[11] = fire_pressed ? 2000 : 1000;
 
 
   // --- DISPLAY CHANGES ---
   // Check for changes and display them on the Serial Monitor
   for (int i = 0; i < NUM_CHANNELS; i++) {
     if (channel_values[i] != previous_channel_values[i]) {
       // Display channel change
       Serial.print("CH");
       Serial.print(i + 1);
       Serial.print(": ");
       Serial.print(channel_names[i]);
       Serial.print(" -> ");
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
   while (!Serial); // Wait for serial port to connect. Needed for native USB port only
   
   Serial.println("\n=== USB Joystick Complete Control Mapper (FIXED) ===");
   Serial.println("Device: Turtle Beach VelocityOne Flightstick");
   Serial.println("Waiting for joystick connection...");
 
   // Initialize all channels to a neutral/default value (1500 for axes, 1000 for buttons)
   for (int i = 0; i < 3; i++) { // Axes
       channel_values[i] = 1500;
       previous_channel_values[i] = 1500;
   }
   for (int i = 3; i < NUM_CHANNELS; i++) { // Buttons
       channel_values[i] = 1000;
       previous_channel_values[i] = 1000;
   }
 
   // Initialize the USB Host Shield
   if (Usb.Init() == -1) {
     Serial.println("USB Host Shield initialization failed! Halting.");
     while (1);
   }
   Serial.println("USB Host Shield Initialized.");
 
   // Set the HID parser
   Hid.SetReportParser(0, &JoyEvents);
 
   Serial.println("\n=== Control Mapper Ready ===");
   Serial.println("Move joystick and press buttons to see channel changes.");
   Serial.println("======================================================");
 }
 
 void loop() {
   // This task must be called continuously to keep the USB stack running.
   Usb.Task();
 }
