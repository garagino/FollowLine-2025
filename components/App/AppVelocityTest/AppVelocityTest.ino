#include "BluetoothSerial.h"
#include <RoboCore_Vespa.h>  // Library for the Vespa microcontroller
VespaMotors motor;  // Vespa Motor Object


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to enable it
#endif

BluetoothSerial SerialBT;
int velocity;
long unsigned tempo;

String prefix;

String getElement(String data, int index) {
  char separator = ' ';
  int found = 0;
  int startIndex = 0, endIndex = -1;
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      startIndex = endIndex + 1;
      endIndex = (i == maxIndex) ? i + 1 : i;
    }
  }

  if (found <= index) {
    return "";
  }

  return data.substring(startIndex, endIndex);
}

String getPrefix(String data) {
  return getElement(data, 0);
}

double getNumber(String data, int index) {
  return atof(getElement(data, index).c_str());
}


void setup() {
  Serial.begin(115200);
  SerialBT.begin("fixaj"); // Nome do dispositivo Bluetooth
  Serial.println("ready");
}

String message;

void loop() {

    if (SerialBT.available()) {
      String message;
      char incomingChar;
      tempo = millis();
      while ((millis() - tempo) < 1000) {
        incomingChar = SerialBT.read();

        message += String(incomingChar);
      }
      Serial.println(message);
    }

}
