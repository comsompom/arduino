/*******************************************************************************
 * USB Device Scanner
 * 
 * HARDWARE:
 * - Arduino UNO
 * - USB Host Shield 2.0
 * 
 * LIBRARIES:
 * - USB Host Shield Library 2.0 by felis
 * 
 * DESCRIPTION:
 * Simple USB device scanner to identify what devices are connected
 * and help troubleshoot USB Host Shield issues.
 *
 ******************************************************************************/

#include <hiduniversal.h>
#include <usbhub.h>
#include <hidboot.h>

// -- USB Host Shield Setup --
USB Usb;
USBHub Hub(&Usb);
HIDUniversal Hid(&Usb);

// Device tracking
unsigned long last_scan = 0;
int device_count = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("=== USB Device Scanner ===");
  Serial.println("Scanning for USB devices...");

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
  
  Serial.println("\n=== Scanner Ready ===");
  Serial.println("Connect USB devices to see them detected");
  Serial.println("=============================================");
}

void loop() {
  // This task must be called continuously to keep the USB stack running.
  Usb.Task();

  // Scan for devices every 3 seconds
  if (millis() - last_scan > 3000) {
    scanForDevices();
    last_scan = millis();
  }
}

void scanForDevices() {
  Serial.println("\n=== USB Device Scan ===");
  
  // Check USB Host Shield status
  Serial.print("USB Host Shield State: ");
  switch (Usb.getUsbTaskState()) {
    case USB_STATE_DETACHED:
      Serial.println("DETACHED");
      break;
    case USB_STATE_ATTACHED:
      Serial.println("ATTACHED");
      break;
    case USB_STATE_POWERED:
      Serial.println("POWERED");
      break;
    case USB_STATE_DEFAULT:
      Serial.println("DEFAULT");
      break;
    case USB_STATE_ADDRESS:
      Serial.println("ADDRESS");
      break;
    case USB_STATE_CONFIGURED:
      Serial.println("CONFIGURED");
      break;
    case USB_STATE_SUSPENDED:
      Serial.println("SUSPENDED");
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
    Serial.println("HID Device: CONNECTED");
    Serial.print("VID: 0x");
    Serial.println(Hid.getVID(), HEX);
    Serial.print("PID: 0x");
    Serial.println(Hid.getPID(), HEX);
    
    // Try to get more device info
    Serial.print("Device Class: ");
    Serial.println(Hid.getDevClass());
    Serial.print("Device SubClass: ");
    Serial.println(Hid.getDevSubClass());
    Serial.print("Device Protocol: ");
    Serial.println(Hid.getDevProtocol());
    
  } else {
    Serial.println("HID Device: NOT FOUND");
  }
  
  // Check USB Hub status
  if (Hub.isReady()) {
    Serial.println("USB Hub: CONNECTED");
  } else {
    Serial.println("USB Hub: NOT FOUND");
  }
  
  Serial.println("================================");
  
  // Provide troubleshooting tips
  if (!Hid.isReady()) {
    Serial.println("Troubleshooting Tips:");
    Serial.println("1. Make sure your device is powered on");
    Serial.println("2. Try unplugging and reconnecting the device");
    Serial.println("3. Check if the device works on a computer");
    Serial.println("4. Try a different USB cable");
    Serial.println("5. Check USB Host Shield power supply (5V)");
    Serial.println("6. Verify all USB Host Shield connections");
    Serial.println("7. Some devices may not be compatible");
    Serial.println();
  }
}
