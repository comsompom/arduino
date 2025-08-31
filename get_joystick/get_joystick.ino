/*******************************************************************************
 * Basic USB Test
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Very basic test to verify USB Host Shield is working
 * and detect any USB devices.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Simple parser class
class BasicParser : public HIDReportParser {
public:
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
    Serial.print("Data received - Length: ");
    Serial.print(len);
    Serial.print(" bytes: ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
};

BasicParser Parser;

void setup() {
  Serial.begin(115200);
  Serial.println("=== Basic USB Test ===");
  Serial.println("Testing USB Host Shield...");

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
  
  // Set the parser
  Hid.SetReportParser(0, &Parser);
  
  Serial.println("\n=== Test Ready ===");
  Serial.println("Connect USB devices to see if they're detected");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Simple status check every 5 seconds
  static unsigned long last_check = 0;
  if (millis() - last_check > 5000) {
    Serial.println("\n=== Status Check ===");
    
    // Check USB state
    Serial.print("USB State: ");
    switch (Usb.getUsbTaskState()) {
      case USB_STATE_DETACHED:
        Serial.println("DETACHED");
        break;
      case USB_STATE_ADDRESSING:
        Serial.println("ADDRESSING");
        break;
      case USB_STATE_CONFIGURING:
        Serial.println("CONFIGURING");
        break;
      case USB_STATE_RUNNING:
        Serial.println("RUNNING");
        break;
      default:
        Serial.println("UNKNOWN");
        break;
    }
    
    // Check if HID device is ready
    if (Hid.isReady()) {
      Serial.println("HID Device: CONNECTED");
      Serial.println("Device is ready and sending data");
    } else {
      Serial.println("HID Device: NOT FOUND");
      Serial.println("No HID device detected");
    }
    
    Serial.println("=====================");
    last_check = millis();
  }
}
