#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

int Rx_pin = 4;
int Tx_pin = 3;
bool newStr = 0;
char symbol = "";
LiquidCrystal_I2C lcd(0x27,20,4); 
String outStr = "";
byte readByte = 0;
SoftwareSerial SerialGPS(Rx_pin, Tx_pin);

void setup()
{
  lcd.init();
  lcd.backlight();
}

void loop()
{
  if (SerialGPS.available() > 0){
    readByte = SerialGPS.read();
    //Serial.print((char)readByte);
    //Serial.print(readByte);
    if (readByte == 36){
      //Serial.println(outStr);
      if (outStr.substring(1,6) == "GPRMC"){
        lcd.setCursor(2,0);
        lcd.print(outStr.substring(7, 13));
        lcd.setCursor(2,1);
        lcd.print(outStr.substring(25, 31))
        //Serial.println(outStr.substring(7, 13));
        //Serial.println(outStr.substring(25, 31));
      }
      outStr = "";
    }
    outStr += (char)readByte;
  }
}