/*******************************************************************************
 * USB Device Detector - Get Joystick Info
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Detects and displays information about connected USB devices,
 * especially useful for identifying joystick devices and their capabilities.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Device tracking
bool device_connected = false;
unsigned long last_device_check = 0;

// Simple parser class to capture device data
class DeviceParser : public HIDReportParser {
public:
  void Parse(USBHID *hid, bool is_rpt_id, uint8_t len, uint8_t *buf) {
    Serial.print("Device Data - Length: ");
    Serial.print(len);
    Serial.print(" bytes: ");
    for (uint8_t i = 0; i < len; i++) {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    // Try to interpret as joystick data
    if (len >= 4) {
      Serial.println("Possible joystick axes:");
      Serial.print("  Axis 1: ");
      Serial.println(buf[0]);
      Serial.print("  Axis 2: ");
      Serial.println(buf[1]);
      Serial.print("  Axis 3: ");
      Serial.println(buf[2]);
      Serial.print("  Axis 4: ");
      Serial.println(buf[3]);
    }
  }
};

DeviceParser Parser;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Device Detector ===");
  Serial.println("Detecting USB devices...");

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
  
  Serial.println("\n=== Device Detector Ready ===");
  Serial.println("Connect USB devices to see their information");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Check for device connection every 3 seconds
  if (millis() - last_device_check > 3000) {
    checkDeviceStatus();
    last_device_check = millis();
  }
}

void checkDeviceStatus() {
  Serial.println("\n=== Device Status Check ===");
  
  // Check USB Host Shield status
  Serial.print("USB Host Shield State: ");
  int state = Usb.getUsbTaskState();
  switch (state) {
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
  
  // Check if HID device is connected
  if (Hid.isReady()) {
    if (!device_connected) {
      Serial.println("*** NEW HID DEVICE CONNECTED ***");
      device_connected = true;
      
      // Try to get device information
      Serial.println("Device Information:");
      Serial.println("  - Device is ready");
      Serial.println("  - Device is sending HID data");
      Serial.println("  - Move device axes/buttons to see data");
    }
  } else {
    if (device_connected) {
      Serial.println("*** HID DEVICE DISCONNECTED ***");
      device_connected = false;
    } else {
      Serial.println("HID Device: NOT FOUND");
      Serial.println("Troubleshooting:");
      Serial.println("  1. Make sure device is powered on");
      Serial.println("  2. Try unplugging and reconnecting");
      Serial.println("  3. Check if device works on computer");
      Serial.println("  4. Try different USB cable");
      Serial.println("  5. Check USB Host Shield power supply");
    }
  }
  
  Serial.println("================================");
}

// Helper function to print hex values
void printHex(uint8_t value) {
  if (value < 0x10) {
    Serial.print("0");
  }
  Serial.print(value, HEX);
}
