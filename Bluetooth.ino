#include "BluetoothSerial.h"
    
BluetoothSerial SerialBT;
     
void setup()
{
  SerialBT.begin("ESP32test");
  Serial.begin(9600);
  delay(1000);
}
     
void loop()
{
  String inputFromOtherSide;
  if (SerialBT.available()) {
    inputFromOtherSide = SerialBT.readString();
    Serial.println("Received: " + inputFromOtherSide);
    SerialBT.println("Answer: " + inputFromOtherSide);
  }
}
