#include <SoftwareSerial.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

int Rx_pin = 4;
int Tx_pin = 3;

String outStr = "";
byte readByte = 0;

LiquidCrystal_I2C lcd(0x27,20,4); 
SoftwareSerial SerialGPS(Rx_pin, Tx_pin);

float main_n = 54.4056999;
float main_e = 25.1515555;

int up_str = 16;
int dn_str = 16;

void setup()
{
  lcd.init();
  lcd.backlight();
  SerialGPS.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  up_str = 16;
  dn_str = 16;
  if (SerialGPS.available() > 0){
    readByte = SerialGPS.read();
    //Serial.print((char)readByte);
    //Serial.print(readByte);
    if (readByte == 36){
      //Serial.println(outStr);
      if (outStr.substring(1,6) == "GPGLL"){
        // Print at lcd Latitude at first string
        int point_index = outStr.indexOf('.', 7);
        String n_pos = outStr.substring(7, 9);
        n_pos += ".";
        n_pos += outStr.substring(9, point_index);
        n_pos += outStr.substring(point_index + 1, 19);
        float n_pos_f = n_pos.substring(0, 10).toFloat();
        float n_diff = abs(main_n - n_pos_f);
        up_str -= n_diff * 10000;
        String up_str_block = "";
        if (up_str < 0 or up_str > 16){
          up_str = 1;
        }
        for (int i = 1; i <= up_str; i++) {
          up_str_block += "#";
        }
        if (up_str < 16){
          for (int i = 1; i <= 16 - up_str; i++) {
            up_str_block += " ";
          }
        }

        lcd.setCursor(0,0);
        //lcd.print("  " + n_pos + "  ");
        //lcd.print((String)n_diff);
        lcd.print(up_str_block);

        // Print at lcd longitude at first string
        point_index = outStr.indexOf('.', 20);
        String e_pos = outStr.substring(20, point_index - 2);
        e_pos += ".";
        e_pos += outStr.substring(point_index - 2, point_index);
        e_pos += outStr.substring(point_index + 1, 33);
        float e_pos_f = e_pos.substring(0, 11).toFloat();
        float e_diff = abs(main_e - e_pos_f);
        dn_str -= e_diff * 10000;
        String dn_str_block = "";
        if (dn_str < 0 or dn_str > 16){
          dn_str = 1;
        }
        for (int i = 1; i <= dn_str; i++) {
          dn_str_block += "#";
        }
        if (dn_str < 16){
          for (int i = 1; i <= 16 - dn_str; i++) {
            dn_str_block += " ";
          }
        }

        lcd.setCursor(0,1);
        //lcd.print("  " + e_pos + "  ");
        lcd.print(dn_str_block);
        //Serial.println(up_str);
        //Serial.println(dn_str);
        //Serial.println(n_diff * 10000);
        //Serial.println(e_diff * 10000);
        Serial.println(n_pos);
        Serial.println(e_pos);
        //Serial.println(outStr.substring(7, 13));
        //Serial.println(outStr.substring(25, 31));
      }
      outStr = "";
    }
    outStr += (char)readByte;
  }
}