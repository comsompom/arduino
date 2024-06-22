#include <SoftwareSerial.h>

int Rx_pin = 4;
int Tx_pin = 3;
bool newStr = 0;
char symbol = "";
String outStr = "";
byte readByte = 0;
SoftwareSerial SerialGPS(Rx_pin, Tx_pin);

void setup()
{
  Serial.begin(9600);
  SerialGPS.begin(9600);
}

void loop()
{
  if (SerialGPS.available() > 0){
    readByte = SerialGPS.read();
    //Serial.print((char)readByte);
    //Serial.print(readByte);
    if (readByte == 36){
      Serial.println(outStr);
      if (outStr.substring(1,5) == "GPRMC"){
        Serial.println("found");
      }
      outStr = "";
    }
    outStr += (char)readByte;
  }
}