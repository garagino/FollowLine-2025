#define DEBUG
#define BT_NAME "I forgot to set a name"

#ifdef DEBUG
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
BluetoothSerial SerialBT;  // Bluetooth Serial instance
#endif




void setup() {

  for (int i= 0; i< 40; i++){
    curves[i].velocity = 30;
    curves[i].breakerStrength = 30;
    curves[i].timeBreaking = 200;
  }


  

  qtr.setTypeRC();  // For QTR-8RC      Sensor pins:
  qtr.setSensorPins((const uint8_t[]){ 21, 19, 5, 16, 22, 23, 18, 17 }, SENSOR_COUNT);

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_MARKER_SENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);

//------------------Encoder-------------------
  pinMode(encoderLeftPin1, INPUT_PULLUP); 
  pinMode(encoderLeftPin2, INPUT_PULLUP);

  pinMode(encoderRightPin1, INPUT_PULLUP); 
  pinMode(encoderRightPin2, INPUT_PULLUP);

  digitalWrite(encoderLeftPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderLeftPin2, HIGH); //turn pullup resistor on

  digitalWrite(encoderRightPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderRightPin2, HIGH); //turn pullup resistor on

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(encoderLeftPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderRightPin1, updateEncoder, CHANGE);

//-------------------------------------------------

#ifdef DEBUG
  if (firstRun) {
    Serial.begin(115200);
    delay(100);

    SerialBT.begin(BT_NAME);  // Bluetooth device name
    firstRun = false;
  }

  SerialBT.println("Start BT communication");

  String btMessage;
  String prefix;

  while (prefix != "end" && digitalRead(PIN_BUTTON) == HIGH) {
    btMessage = receiveBtMessage();
    prefix = getPrefix(btMessage);

    if (prefix == "pid") {
      Kp = getNumber(btMessage, 1);
      Ki = getNumber(btMessage, 2);
      Kd = getNumber(btMessage, 3);
    } else if (prefix == "spe") {
      maxSpeed = getNumber(btMessage, 1);
    } else if (prefix == "mrk"){
        markersDistance[markersSetupIndex] = getNumber(btMessage, 1);
        markersSetupIndex++;
    }else if (prefix == "for") {
      forwardSpeed = getNumber(btMessage, 1);
    } else if (prefix == "tim") {
      startMakerChecker = getNumber(btMessage, 1);
    } else if (prefix == "err") {
      marginError = getNumber(btMessage, 1);
    } else if (prefix == "pri") {
      printParameters();
    } else if (prefix == "end") {
      break;
    } else if (prefix == "lim"){
      if ( limiter == false){
        limiter = true;
        SerialBT.println("Limiter ativado!");
      } else{
        limiter = false;
        SerialBT.println("Limiter desativado!");
      }
    } else {
      SerialBT.println("This command doesn't exists!");
    }
  }

  printParameters();

  SerialBT.println("Start Calibration...");

  delay(500);
#endif

  // Calibration
  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {  // Calibrates until the button is pressed
    qtr.calibrate();
    //------------------Encoder-------------------
    encoderValue = 0;
    //-------------------------------------
  }
  digitalWrite(PIN_LED, LOW);

#ifdef DEBUG
  // Print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibrationOn.minimum[i]);
    SerialBT.print(' ');
  }
  SerialBT.println();

  // Print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    SerialBT.print(qtr.calibrationOn.maximum[i]);
    SerialBT.print(' ');
  }
  SerialBT.println();
#endif

  delay(2000);  // Start loop after 2 seconds
  initialTime = millis();
}