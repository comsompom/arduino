/*
 * Copyright (c) 2012-2013, Oleg Mazurov, Circuits@Home
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN

 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * USB Descriptor Master
 *
 * This sketch displays all configuration, interface, and endpoint descriptors
 * for a given USB device.
 *
 * It is intended to be used as a debugging tool to display all available
 * information about any USB device.
 *
 * Created by Oleg Mazurov, 2012
 *
 * Modified by Bert Frees, 2013
 *  - Use the Serial port on the Arduino instead of a SPI-driven LCD screen.
 *
 */

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Usb.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

void print_hex(int v, int num_places);
void print_string(const char *str);

class DevDesc : public USBDeviceConfig {
public:
    uint8_t ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed);
protected:
    void PrintAllDescriptors(uint8_t addr);
};

uint8_t DevDesc::ConfigureDevice(uint8_t parent, uint8_t port, bool lowspeed) {
    uint8_t rcode;
    uint8_t len;
    Address.parent = parent;
    Address.port = port;
    Address.lowspeed = lowspeed;
    /* requesting device descriptor */
    rcode = pUsb->getDevDescr(1, 0, 0x12, (uint8_t*) & buf); //get device descriptor
    if (rcode) {
        Serial.println("Can't get device descriptor. Error: ");
        print_hex(rcode, 8);
        Serial.println("");
        return rcode;
    }
    PrintAllDescriptors(1);
    return 0;
}

void DevDesc::PrintAllDescriptors(uint8_t addr) {
    uint8_t rcode;
    uint8_t desc_len;
    uint8_t desc_type;
    uint16_t total_len;
    const char *string_ptr;
    rcode = pUsb->getConfDescr(addr, 0, 4, 0, buf);
    if (rcode) {
        Serial.print("Error getting config descriptor 1st time. Error code: ");
        print_hex(rcode, 8);
        Serial.println("");
        return;
    }
    total_len = buf[2] | (buf[3] << 8); //total length of configuration descriptor equals to the value of wTotalLength field
    Serial.print("Total length: ");
    print_hex(total_len, 16);
    Serial.println("");
    rcode = pUsb->getConfDescr(addr, 0, total_len, 0, buf); //get the whole descriptor
    if (rcode) {
        Serial.print("Error getting config descriptor 2nd time. Error code: ");
        print_hex(rcode, 8);
        Serial.println("");
        return;
    }
    uint16_t curr_ptr = 0;
    while (curr_ptr < total_len) {
        desc_len = buf[curr_ptr];
        desc_type = buf[curr_ptr + 1];
        switch (desc_type) {
            case (USB_DESCRIPTOR_CONFIGURATION):
                string_ptr = "Configuration descriptor:";
                break;
            case (USB_DESCRIPTOR_INTERFACE):
                string_ptr = "Interface descriptor:";
                break;
            case (USB_DESCRIPTOR_ENDPOINT):
                string_ptr = "Endpoint descriptor:";
                break;
            case (HID_DESCRIPTOR_HID):
                string_ptr = "HID descriptor:";
                break;
            case (HID_DESCRIPTOR_REPORT):
                string_ptr = "HID report descriptor:";
                break;
            case (CS_INTERFACE):
                string_ptr = "Class-specific interface descriptor:";
                break;
            case (CS_ENDPOINT):
                string_ptr = "Class-specific endpoint descriptor:";
                break;
            default:
                string_ptr = "Unknown descriptor:";
                break;
        }//switch( desc_type...
        Serial.println(string_ptr);
        for (uint8_t i = 0; i < desc_len; i++) {
            print_hex(buf[curr_ptr + i], 8);
            Serial.print(" ");
        }
        Serial.println("");
        curr_ptr += desc_len; //advance to next descriptor
    }//while( curr_ptr...
}

USB Usb;
DevDesc Dev;

void setup() {
    Serial.begin(115200);
#if !defined(__MIPSEL__)
    while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
    Serial.println("Start");
    if (Usb.Init() == -1)
        Serial.println("OSCOKIRQ failed to assert");
    delay(200);
    Dev.SetAddress(1);
    Usb.SetAddress(0);
    Usb.SetEventHandler(&Dev);
}

void loop() {
    Usb.Task();
}

void print_hex(int v, int num_places) {
    int mask = 0, n, num_nibbles, digit;
    for (n = 1; n <= num_places; n++) {
        mask = (mask << 1) | 0x0001;
    }
    v = v & mask; // truncate v to specified number of places
    num_nibbles = num_places / 4;
    if ((num_places % 4) != 0) {
        ++num_nibbles;
    }
    do {
        digit = ((v >> (num_nibbles - 1) * 4)) & 0x0f;
        Serial.print(digit, HEX);
    } while (--num_nibbles);
}