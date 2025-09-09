/*******************************************************************************
 * USB Joystick to PPM Signal Generator - CORRECTED & STABILIZED
 *
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * - Turtle Beach VelocityOne Flightstick
 * - 3.5mm Mono Jack (for PPM output)
 *
 * WIRING:
 * - Arduino Pin 2 -> 3.5mm Jack TIP (PPM Signal)
 * - Arduino GND   -> 3.5mm Jack SLEEVE (Ground)
 *
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 *
 * DESCRIPTION:
 * This sketch reads data from a Turtle Beach VelocityOne Flightstick and
 * generates a stable 12-channel PPM signal. The throttle uses discrete
 * percentage values with NO SMOOTHING for precise control.
 *
 * CORRECTIONS:
 * - Efficient mathematical throttle mapping using map() function (like elevator/aileron).
 * - Smooth continuous mapping with 25μs discrete rounding for precise control.
 * - Much more efficient than conditional statements - uses simple math operations.
 *
 ******************************************************************************/

 #include <hiduniversal.h>
 #include <usbhub.h>
 
 // -- PPM Configuration --
 #define PPM_PIN 2           // Digital pin for PPM output
 #define NUM_CHANNELS 12     // Total number of channels
 #define PPM_FRAME_LENGTH 22500 // 22.5ms total frame time
 #define PPM_PULSE_LENGTH 300   // 0.3ms pulse length
 #define PPM_CHANNEL_MIN 1000   // Minimum channel value (us)
 #define PPM_CHANNEL_MAX 2000   // Maximum channel value (us)
 #define PPM_CHANNEL_MID 1500   // Center channel value (us)
 
 // Array to hold the values for each channel in microseconds
 uint16_t ppm_channels[NUM_CHANNELS];
 
 // -- USB Host Shield Setup --
 USB Usb;
 USBHub Hub(&Usb);
 HIDUniversal Hid(&Usb);
 
 // Channel names for display
 const char* channel_names[NUM_CHANNELS] = {
   "Fire", "Throttle", "Elevator", "Aileron", "Button1", "Button2",
   "Button3", "Button4", "Button5", "Button6", "Button7", "Button8"
 };
 
 // This class is where we parse the joystick's raw data
 class JoystickEvents : public HIDReportParser {
 public:
   JoystickEvents();  // Constructor
   void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
 };
 
 // Constructor for our parser
 JoystickEvents::JoystickEvents() {}
 
 // This function is called by the USB Host library every time the joystick sends a new data packet
 void JoystickEvents::Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
   // We only care about data packets with a length of 51
   if (len != 51) {
     return;
   }
 
   // --- AXES ---
   // Aileron (X-axis): bytes 1 & 2 (16-bit) -> Channel 4
   uint16_t raw_aileron = (buf[2] << 8) | buf[1];
   ppm_channels[3] = map(raw_aileron, 0x0000, 0xFFFF, PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);
 
   // Elevator (Y-axis): bytes 3 & 4 (16-bit) -> Channel 3
   uint16_t raw_elevator = (buf[4] << 8) | buf[3];
   // The elevator axis is often inverted on flight sticks
   ppm_channels[2] = map(raw_elevator, 0x0000, 0xFFFF, PPM_CHANNEL_MAX, PPM_CHANNEL_MIN);
 
 
    // =========================================================================
    // === EFFICIENT THROTTLE MAPPING - MATHEMATICAL APPROACH =================
    // =========================================================================
    // Throttle (Left Lever): bytes 11 & 12 (16-bit) -> Channel 2
    // CORRECTED: buf[11] is LSB, buf[12] is MSB (little-endian)
    uint16_t raw_throttle = (buf[12] << 8) | buf[11];
    
    // Efficient mathematical mapping with discrete steps (like elevator/aileron)
    // Map raw throttle (0x0000-0xFFFF) to PPM range (1000-2000) with 25μs steps
    uint16_t mapped_throttle = map(raw_throttle, 0x0000, 0xFFFF, 1000, 2000);
    
    // Round to nearest 25μs increment for discrete values
    ppm_channels[1] = ((mapped_throttle + 12) / 25) * 25;
    
    // Ensure values stay within valid range
    ppm_channels[1] = constrain(ppm_channels[1], 1000, 2000);
    // =========================================================================
 
 
   // Debug: Show only throttle mapped value when it changes
   static uint16_t last_throttle_state = 0;
   if (ppm_channels[1] != last_throttle_state) {
     Serial.print("THROTTLE: ");
     Serial.print(ppm_channels[1]);
     Serial.println("us");
     last_throttle_state = ppm_channels[1];
   }
 
 
   // --- BUTTONS ---
   // Buttons B1-B8 are a bitmask in byte 18 -> Channels 5-12
   uint8_t button_byte = buf[18];
   for (int i = 0; i < 8; i++) {
     bool button_state = (button_byte & (1 << i)) != 0;
     ppm_channels[i + 4] = button_state ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;
   }
 
   // FIRE BUTTON (byte 20, bit 1) -> Channel 1
   bool fire_pressed = (buf[20] & 0x02) != 0;
   ppm_channels[0] = fire_pressed ? PPM_CHANNEL_MAX : PPM_CHANNEL_MIN;
 
   // Ensure all values are within the valid PPM range just in case
   for (int i = 0; i < NUM_CHANNELS; i++) {
     ppm_channels[i] = constrain(ppm_channels[i], PPM_CHANNEL_MIN, PPM_CHANNEL_MAX);
   }
 
   // --- SERIAL MONITOR DEBUGGING ---
   // This will print the values only when they change, preventing spam.
   static uint16_t previous_values[NUM_CHANNELS] = {0};
   bool changed = false;
   for (int i = 0; i < NUM_CHANNELS; i++) {
     if (ppm_channels[i] != previous_values[i]) {
       changed = true;
       break;
     }
   }
   if (changed) {
       for (int i = 0; i < NUM_CHANNELS; i++) {
           if (ppm_channels[i] != previous_values[i]) {
               Serial.print(channel_names[i]);
               Serial.print(" (CH");
               Serial.print(i + 1);
               Serial.print("): ");
               Serial.print(ppm_channels[i]);
               Serial.println("us");
               previous_values[i] = ppm_channels[i];
           }
       }
   }
 }
 
 // Create an instance of our parser class
 JoystickEvents JoyEvents;
 
 void setup() {
   Serial.begin(115200);
   while (!Serial);
 
   Serial.println("\n=== USB Joystick to PPM Signal Generator (Corrected) ===");
 
   // Initialize all PPM channels to safe default values
   ppm_channels[0] = PPM_CHANNEL_MIN;  // Fire (released)
   ppm_channels[1] = PPM_CHANNEL_MIN;  // Throttle (minimum)
   ppm_channels[2] = PPM_CHANNEL_MID;  // Elevator (center)
   ppm_channels[3] = PPM_CHANNEL_MID;  // Aileron (center)
   for (int i = 4; i < NUM_CHANNELS; i++) { // Buttons B1-B8 (released)
     ppm_channels[i] = PPM_CHANNEL_MIN;
   }
 
   pinMode(PPM_PIN, OUTPUT);
   digitalWrite(PPM_PIN, LOW);
 
   if (Usb.Init() == -1) {
     Serial.println("USB Host Shield initialization failed! Halting.");
     while (1);
   }
   Serial.println("USB Host Shield Initialized. Waiting for joystick...");
 
   Hid.SetReportParser(0, &JoyEvents);
 }
 
 void loop() {
   Usb.Task();
   generatePpmSignal();
 }
 
 // Standard PPM Signal Generation Function
 void generatePpmSignal() {
   static unsigned long last_frame_time = 0;
 
   if (micros() - last_frame_time >= PPM_FRAME_LENGTH) {
     last_frame_time = micros();
 
     for (int i = 0; i < NUM_CHANNELS; i++) {
       // Low pulse (separator)
       digitalWrite(PPM_PIN, LOW);
       delayMicroseconds(PPM_PULSE_LENGTH);
 
       // High pulse (channel value)
       digitalWrite(PPM_PIN, HIGH);
       delayMicroseconds(ppm_channels[i] - PPM_PULSE_LENGTH);
     }
     // Final low pulse to end the channel train
     digitalWrite(PPM_PIN, LOW);
 
     // The rest of the frame time is the sync pulse (idle low)
   }
 }
