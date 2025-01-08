/****************************************************************
* Folow Line
* CESAR School
* 
* Line Follower PID with the microcontroller Vespa 
* from RoboCore and the Pololu's QTR-8RC sensor
****************************************************************/

#define DEBUG
#define BT_NAME "vin-A"
// Names: Mutuca | Motoneta | Van Dyne


#ifdef DEBUG
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;  // Bluetooth Serial instance
#endif

#include <RoboCore_Vespa.h>  // Library for the Vespa microcontroller
#include <QTRSensors.h>      // Library for the QTR-8A or the QTR-8RC

VespaMotors motor;  // Vespa Motor  Object
QTRSensors qtr;     // QTR Sensor

// Set button and led pins
const uint8_t PIN_BUTTON = 35;
const uint8_t PIN_LED = 15;
const uint8_t PIN_MARKER_SENSOR = 25;

//Setup of the module of sensors
const uint8_t SENSOR_COUNT = 8;       // The number of sensors, which should match the length of the pins array
uint16_t sensorValues[SENSOR_COUNT];  // An array in which to store the calibrated sensor readings

// Maximum line position, considering the amount of sensors.
const long MAX_POSITION = (SENSOR_COUNT - 1) * 1000;

//Marker sensor variables
unsigned long startMakerChecker = 35500L;
unsigned long initialTime;

unsigned long startCurveBreak;
bool curveBreak = false;


// Limit value of the margin of error
int marginError = 20;

bool firstRun = true;

//------------------PID Control-------------------
float p = 0, i = 0, d = 0, pid = 0, error = 0, lastError = 0;

float Kp = 0.71;
float Ki = 0.0001;
float Kd = 4.3;

int turnSpeed = 100;

int maxSpeed = 100;
int integralLimit = 200;
int lSpeed, rSpeed;

const bool LINE_BLACK = false;

bool limiter = true;

//------------------Encoder-------------------

float distanceLeftMotor;          //Calculates the distance of te left motor in mm
float distanceRightMotor;         //Calculates the distance of te Right motor in mm
float distanceAverage;            //Calculates the average distance in mm
float multEncoder;                

float maxDistance = 565;              //Maximum distance 



int timeDeltaCurve = 0;            // Controls the amount of times the curve interrupt is activated
int curveMarkCount;                // Counts the amount of times the robot has passed by a Curve marker
int curveCount;                    // Counts the amount of times the robot has passed by the first marker of a curve
int endLineCount;                  // Counts the amount of times the robot has passed by a End marker


int timeDeltaBreaker = 0;               // breaking time  
int timeBreaker = 100;               // breaking time  
int breaker = 90;                    // breaker strengh

int encoderLeftPin = 32; //Encoder Output 'A' must connected with intreput pin of arduino.
int encoderRightPin = 33; //Encoder Output 'B' must connected with intreput pin of arduino.

int  encoderValueLeft = 0;
int  encoderValueRight = 0;


//-------------------------------------------------

void setup() {
  qtr.setTypeRC();  // For QTR-8RC      Sensor pins:
  qtr.setSensorPins((const uint8_t[]){ 21, 19, 5, 16, 22, 23, 18, 17 }, SENSOR_COUNT);

  pinMode(PIN_BUTTON, INPUT);
  pinMode(PIN_MARKER_SENSOR, INPUT);
  pinMode(PIN_LED, OUTPUT);

  multEncoder = 0.0526;

 
  pinMode(encoderLeftPin, INPUT_PULLUP); 
  pinMode(encoderRightPin, INPUT_PULLUP); 
  digitalWrite(encoderLeftPin, HIGH); //turn pullup resistor on
  digitalWrite(encoderRightPin, HIGH); //turn pullup resistor on



  attachInterrupt(encoderLeftPin, updateEncoderLeft, CHANGE);

  attachInterrupt(encoderRightPin, updateEncoderRight, CHANGE);

  attachInterrupt(PIN_MARKER_SENSOR, curveBreaker, HIGH);

  // attachInterrupt(PIN_MARKER_SENSOR, countEndLine, HIGH);


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

    if (prefix == "turn") {
      turnSpeed = getNumber(btMessage, 1);
    }

    if (prefix == "pid") {
      Kp = getNumber(btMessage, 1);
      Ki = getNumber(btMessage, 2);
      Kd = getNumber(btMessage, 3);
    } else if (prefix == "spe") {
      maxSpeed = getNumber(btMessage, 1);
    } else if (prefix == "turn") {
      turnSpeed = getNumber(btMessage, 1);
    } else if (prefix == "tim") {
      startMakerChecker = getNumber(btMessage, 1);
    } else if (prefix == "err") {
      marginError = getNumber(btMessage, 1);
    } else if (prefix == "dis") {
      maxDistance = getNumber(btMessage, 1);
    } else if (prefix == "mul") {
      multEncoder = getNumber(btMessage, 1);
    } else if (prefix == "bre") {
      breaker = getNumber(btMessage, 1);
    } else if (prefix == "tbe") {
      timeBreaker = getNumber(btMessage, 1);
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

    encoderValueLeft = 0;
    encoderValueRight = 0;

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

void loop() {
  // readSensors() returns the line position between 0 and `MAX_POSITION`.
  // error is a re-map from -1000 to 1000 range.


  error = map(readSensors(), 0, MAX_POSITION, -1000, 1000);





  // Calculate PID
  p = error;
  i = i + error;
  if (limiter == true){
    i = constrain(i, -integralLimit, integralLimit); // Limita o valor do integral
  }
  d = error - lastError;
  pid = (Kp * p) + (Ki * i) + (Kd * d);
  lastError = error;

  // Control Motors
  lSpeed = maxSpeed + pid;
  rSpeed = maxSpeed - pid;

  lSpeed = constrain(lSpeed, -maxSpeed, maxSpeed);
  rSpeed = constrain(rSpeed, -maxSpeed, maxSpeed);

  curveCount = curveMarkCount % 2;
  


  if (curveCount == 1){
    timeDeltaBreaker = millis() - 1;
    Serial.println("teste2");
    while((millis() - timeDeltaBreaker) < timeBreaker){
      pid = 0.0;
      motor.backward(breaker);
      curveCount = 0;
      curveMarkCount++;
      Serial.println(timeDeltaBreaker);
    }
  }




  if (markerChecker()) {  // Count the markers and stop the robot when reach a certain number
    SerialBT.print("Average distance  ");
    SerialBT.println(distanceAverage);
    motor.stop();



#ifdef DEBUG
    SerialBT.print(">> Timelapse: "); 
    SerialBT.print(millis() - initialTime);
    SerialBT.println(" seconds");
#endif
    setup();
  } else if (error >= -marginError && error <= marginError) {  // If the error is within the MARGIN_ERROR, move on
    motor.turn(turnSpeed, turnSpeed);
  } else {  // If the error is outside the error range, continue doing PID
    motor.turn(lSpeed, rSpeed);
  }
}

int readSensors() {
  if (LINE_BLACK) {
    return qtr.readLineBlack(sensorValues);
  } else {
    return qtr.readLineWhite(sensorValues);
  }
}

/**
  Verifies if there is a end line after a set time

  @return `true` if the end line was detected.
*/
bool markerChecker() {

  distanceLeftMotor = encoderValueLeft*multEncoder;
  distanceRightMotor = encoderValueRight*multEncoder;

  distanceAverage = (distanceLeftMotor/2.0) + (distanceRightMotor/2.0);

  static int i = 0;
  if (startMakerChecker < millis() - initialTime) {
    // if (analogRead(PIN_MARKER_SENSOR) < 2000) {
    return true;
    // }
  }
  if (distanceAverage > maxDistance) {
    return true;
  }

  return false;
}

#ifdef DEBUG

/**
  Returns all stream of data sent over bluetooth until the
  button is pressed.

  @return `String` with the message sent by the bluetooth device
*/
String receiveBtMessage() {
  String message;
  char incomingChar;

  digitalWrite(PIN_LED, HIGH);
  while (digitalRead(PIN_BUTTON) == HIGH) {
    if (SerialBT.available()) {
      incomingChar = SerialBT.read();

      if (incomingChar == '\n') break;

      message += String(incomingChar);
    }
  }
  digitalWrite(PIN_LED, LOW);

  message.trim();
  return message;
}

String getPrefix(String data) {
  return getElement(data, 0);
}

double getNumber(String data, int index) {
  return atof(getElement(data, index).c_str());
}

/**
  Returns a sub-string in the `String` data, in the index
  position.

  @param `data` String with the message
  @param `index` Position of the element to be returned
  @return `String` sub-string in the indicated position. If there is
  no value at this position, it returns empty string.
*/
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

void printParameters() {
  SerialBT.println("Configured parameters:");
  SerialBT.print(">> P: ");
  SerialBT.print(Kp, 4);
  SerialBT.print(" | I: ");
  SerialBT.print(Ki, 4);
  SerialBT.print(" | Kd: ");
  SerialBT.println(Kd, 4);

  SerialBT.print(">> Speed: ");
  SerialBT.println(maxSpeed);

  SerialBT.print(">> Time delay: ");
  SerialBT.println(startMakerChecker);

  SerialBT.print(">> Margin Error: ");
  SerialBT.println(marginError);

  SerialBT.print(">> Limitador: ");
  SerialBT.println(limiter);

  SerialBT.print(">> turnSpeed: ");
  SerialBT.println(turnSpeed);

  SerialBT.print(">> Max Distance: ");
  SerialBT.println(maxDistance);

  SerialBT.print(">> Mult Encoder: ");
  SerialBT.println(multEncoder);


  SerialBT.print(">> breaker: ");
  SerialBT.println(breaker);


}

#endif





void updateEncoderLeft()
  {
    encoderValueLeft ++;
  }

void updateEncoderRight()
  {
    encoderValueRight ++;
  }

void curveBreaker (){
  
  if (millis() - timeDeltaCurve > 100) {
    Serial.println("teste");
    timeDeltaCurve = millis();
    curveMarkCount++;
  }
}

void countEndLine(){
  endLineCount++;
}