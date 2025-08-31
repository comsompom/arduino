/*
 * Universal HID Joystick Data Inspector
 * 
 * This sketch connects to any generic HID device (like a flightstick) and
 * prints the raw data report to the Serial Monitor. This allows you to "decode"
 * the joystick's data format by moving axes and pressing buttons and observing
 * which bytes change.
 * 
 * REQUIRED HARDWARE:
 * 1. Arduino Uno
 * 2. USB Host Shield 2.0
 * 3. A POWERED USB HUB to power the joystick!
 * 
 * Connections:
 * Joystick -> Powered Hub -> USB Host Shield -> Arduino
 * 
 * HOW TO USE:
 * 1. Upload this sketch.
 * 2. Open the Serial Monitor and set the baud rate to 115200.
 * 3. Connect the joystick (through the powered hub).
 * 4. A "Device Connected" message should appear.
 * 5. Move one axis (e.g., X-axis left and right). Watch the Serial Monitor to see which byte value changes. Note it down.
 * 6. Do the same for the Y-axis, Z-twist, throttle, etc.
 * 7. Press buttons one by one. Buttons are often represented as individual bits within a byte (a "bitmask").
 * 
 * Based on the HIDUniversal example from the USB Host Shield Library 2.0 by felis
 */

#include <hiduniversal.h>

// Create an instance of the HIDUniversal class
USB Usb;
HIDUniversal Hid(&Usb);

// This is a custom report parser that will print the raw data
class MyReportParser : public HIDReportParser {
  public:
    MyReportParser();
    void Parse(HIDUniversal *hid, bool is_rpt_id, uint8_t len, uint8_t *buf);
};

MyReportParser::MyReportParser() {}

// This is the core function. It gets called every time the joystick sends a new data packet.
void MyReportParser::Parse(HIDUniversal *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
  // Print a header to know a new data packet has arrived
  Serial.print("Data Packet (Length: ");
  Serial.print(len);
  Serial.print("): ");

  // Print each byte of the data packet in HEX format.
  // HEX is often easier to read for this kind of data.
  for (uint8_t i = 0; i < len; i++) {
    if (buf[i] < 0x10) {
      Serial.print("0"); // Add leading zero for single-digit hex values
    }
    Serial.print(buf[i], HEX);
    Serial.print(" ");
  }
  Serial.println(); // Newline for the next packet
}


// Create an instance of our custom parser
MyReportParser myParser;

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect (useful for Leonardo, etc.)
  
  Serial.println("Start");

  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
  }

  delay(200);

  // Set our custom parser to be used by the HIDUniversal driver
  Hid.SetReportParser(0, &myParser);
}

void loop() {
  // This is required to keep the USB stack running
  Usb.Task();
}