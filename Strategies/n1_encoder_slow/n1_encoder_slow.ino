/**********************
* Folow Line
* CESAR School
* 
* Line Follower PID with the microcontroller Vespa 
* from RoboCore and the Pololu's QTR-8RC sensor
*
* This project decreases robot speed at encoder
* markings
**********************/

/*
* Código teve a estratégia de diminuir a velocidade após 
* uma distância percorrida
* 
*  
* Para isso, foi feito uma funcao durante o loop que retorna a posicao do robô ao mandar uma mensagem vazia via bluetooth
*
* A parada final é baseada no sensor Lateral da Direita
*/

#include "Arduino.h"          // Library for the task manager
#include "BluetoothSerial.h"  // Library for the Buetooth module
#include <RoboCore_Vespa.h>   // Library for the Vespa microcontroller
#include <QTRSensors.h>       // Library for the QTR-8A or the QTR-8RC

// ----------------------- Dual core tasks  ----------------------- 
TaskHandle_t Task0;
TaskHandle_t Task1;

//----------------------------- Bluetooth --------------------------
// Defines Device's Bluetooth name
#define DEBUG
#define BT_NAME "N1"
#ifdef DEBUG

// Makes sure bluetooth is working
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run make menuconfig to and enable it
#endif
BluetoothSerial SerialBT;  // Bluetooth Serial instance
#endif

//----------------------- Vespa ----------------------
VespaMotors motor;  // Vespa Motor  Object
QTRSensors qtr;     // QTR Sensor

// Set button and led pins
const uint8_t PIN_BUTTON = 35;
const uint8_t PIN_LED = 15;
const uint8_t PIN_MARKER_SENSOR = 36;

//----------------------- QTR Sensors ----------------------
//Setup of the module of sensors
const uint8_t SENSOR_COUNT = 8;       // The number of sensors, which should match the length of the pins array
uint16_t sensorValues[SENSOR_COUNT];  // An array in which to store the calibrated sensor readings


// Maximum line position, considering the amount of sensors.
const long MAX_POSITION = (SENSOR_COUNT - 1) * 1000;

//Marker sensor variables
unsigned long startMakerChecker = 39500L;
unsigned long initialTime;

// Limit value of the margin of error
int marginError = 20;
bool firstRun = true; 

// QTR control for if it is above black
const bool LINE_BLACK = false;  

//-----------------------Curve Sensor----------------------
#define SENSOR_PIN_1 36   // Pino ADC para o sensor de linha (deve ser um pino analógico do ESP32)
#define SENSOR_PIN_2 39   // Pino ADC para o sensor de linha (deve ser um pino analógico do ESP32)

int curveCount = 0;                 // Curve counter
bool curveSensorWhite = false;      // if the sensor is above the marker

//---------------------------PID Control-------------------------------------
float p = 0, i = 0, d = 0, pid = 0, error = 0, lastError = 0;

float Kp = 0.412;
float Ki = 0.0001;
float Kd = 2.1;

// O turnSpeed virou forwardSpeed por motivos de coesão e coerência com o código. Prefixo 'for'.
int forwardSpeed = 100;
int maxSpeed = 100;
int integralLimit = 200;
int lSpeed, rSpeed;

bool limiter = true;
bool endTrack = false;

//------------------Encoder-------------------
volatile long encoderValueLeft = 0; // Raw encoder value
volatile long encoderValueRight = 0; // Raw encoder value
bool countEnabled = true;
int stoppingPoint = 22300;

int encoderLeftPin1 = 33;   // Encoder Output 'A' for left encoder
int encoderLeftPin2 = 32;   // Encoder Output 'B' for left encoder
int encoderRightPin1 = 25;  // Encoder Output 'A' for left encoder
int encoderRightPin2 = 26;  // Encoder Output 'B' for left encoder

volatile long encoderRightValue = 0;   // Variable to store encoder count
volatile long encoderLeftValue = 0;  // Variable to store encoder count


//----------------------- Functions ----------------------- 

int readSensors() {
  if (LINE_BLACK) {
    return qtr.readLineBlack(sensorValues);
  } else {
    return qtr.readLineWhite(sensorValues);
  }
}

/**
  Verifies if there is a end line after a set time

  @return true if the end line was detected.
*/

// After (startMakerChecker) time, beggins to look after the right mark 

#ifdef DEBUG

/**
  Returns all stream of data sent over bluetooth until the
  button is pressed.

  @return String with the message sent by the bluetooth device
*/

// returns chars into string
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

// Gets prefix 
String getPrefix(String data) {
  return getElement(data, 0);
}


double getNumber(String data, int index) {
  return atof(getElement(data, index).c_str());
}

/**
  Returns a sub-string in the String data, in the index
  position.

  @param data String with the message
  @param index Position of the element to be returned
  @return String sub-string in the indicated position. If there is
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

void updateRightEncoder() {
  if (digitalRead(encoderRightPin1) == digitalRead(encoderRightPin2)) {
    encoderValueRight++;
  } else {
    encoderValueRight--;
  }
}

void updateLeftEncoder(){
  if (digitalRead(encoderLeftPin1) == digitalRead(encoderLeftPin2)) {
  encoderValueLeft++;
  }
  else {
    encoderValueLeft--;
  }
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

  SerialBT.print(">> Forward Speed: ");
  SerialBT.println(forwardSpeed);
}

#endif

void loop0(void * parameter) {
    
    int mediaEncoder;
    for (;;) {   
      mediaEncoder = (encoderValueLeft + encoderValueRight)/2;  
      if (SerialBT.available()) {
        SerialBT.println (mediaEncoder);
        SerialBT.read();
      }
      // implemaentacao temporaria usando hardcode, definimos os valores 
      // dos pontos e definimos a velocidade maxima em erro menor que a margem (forwardSpeed) 
      // e a maior velocidade em regime de PID (maxSpeed)
      // diminuimos a velocidade do robo em pontos específicos da pista
      // apos testes vamos implementar os valores em um vetor 
      
      // if(mediaEncoder % 100 == 0 && mediaEncoder != 0){
      //   SerialBT.println(mediaEncoder);
      // }
      
      if ((mediaEncoder > 0) && (mediaEncoder <= 2200)){  
        maxSpeed = 100;
        forwardSpeed = 85;
        // trecho 0 | start -> p1 | AZUL
      }else if ((mediaEncoder > 2200) && (mediaEncoder <= 2600)) {        
        maxSpeed = 100;
        forwardSpeed = 95;
        // trecho 1 |  p1 -> p2 | VERMELHO
      }else if ((mediaEncoder > 2600) && (mediaEncoder <= 3300)) {        
        maxSpeed = 100;
        forwardSpeed = 70;
        // trecho 2 | p1 -> p2 | CIANO
      }else if ((mediaEncoder > 3300) && (mediaEncoder <= 8900)){       
        maxSpeed = 100;
        forwardSpeed = 90;
        // trecho 3 | p2 -> p3 | ROSA
      }else if ((mediaEncoder > 8900) && (mediaEncoder <= 15000)){
        maxSpeed = 100;
        forwardSpeed = 90;
        // trecho 4 | p3 -> p4 | VERDE
      }else if ((mediaEncoder > 15000) && (mediaEncoder <= 16000)){
        maxSpeed = 100;
        forwardSpeed = 95;
        // trecho 5 | p4 -> p5 | BRANCO
      }else if ((mediaEncoder > 16000) && (mediaEncoder <= 17000)){
        maxSpeed = 100;
        forwardSpeed = 70;
        // trecho 6 | p4 -> p5 | AMARELO
      }else if ((mediaEncoder > 17000) && (mediaEncoder <= 18500)){
        maxSpeed = 100;
        forwardSpeed = 95; 
        // trecho 7 | p4 -> p5 | VERMELHO 2
      }else if ((mediaEncoder > 18500) && (mediaEncoder <= 22050)){
        maxSpeed = 100;
        forwardSpeed = 95;
        // trecho 8 | p5 -> end | LARANJA
      }else if (mediaEncoder > stoppingPoint){
        SerialBT.print(">> end?: ");
        SerialBT.println(mediaEncoder);
        endTrack = true;
      }   
    }
                                                                                         // Or when finds 
  vTaskDelay(10);
}

// IronCUP - ~31s

void loop1(void * parameter) {
	for (;;) {

        // readSensors() returns the line position between 0 and MAX_POSITION.
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
#ifdef DEBUG
        //SerialBT.print(">> Timelapse: "); 
        //SerialBT.print(millis() - initialTime);
        //SerialBT.println(" seconds");
#endif
        if (endTrack){
            for (int i = 0; i < 10000; i++){ // se deseja aumentar um pouco o tempo da inércia do robô parar até ele de fato parar de andar abaixe o tamanho do loop.
                motor.turn(-50, -50);
            }
            motor.stop();
            ESP.restart(); 
        }else{    
            if (error >= -marginError && error <= marginError) {  // If the error is within the MARGIN_ERROR, move on
                motor.turn(forwardSpeed, forwardSpeed);
            }else{  // If the error is outside the error range, continue doing PID
                motor.turn(lSpeed, rSpeed);
            }    
	    }
        vTaskDelay(10);
    }
}

void setup()
{
  // noInterrupts();                                 // stop interrupts from accurring
	Serial.begin(115200);
	Serial.println("Setup started.");

	xTaskCreatePinnedToCore(
			loop0, /* Function to implement the task */
			"Task0", /* Name of the task */
			1000, /* Stack size in words */
			NULL, /* Task input parameter */
			0, /* Priority of the task */
			&Task0, /* Task handle. */
			0); /* Core where the task should run */

	xTaskCreatePinnedToCore(
			loop1, /* Function to implement the task */
			"Task1", /* Name of the task */
			2000, /* Stack size in words */
			NULL, /* Task input parameter */
			0, /* Priority of the task */
			&Task1, /* Task handle. */
			1); /* Core where the task should run */

	Serial.println("Setup completed.");

  // Suspends both Tasks to start after the calibration state
  vTaskSuspend(Task0); 
  vTaskSuspend(Task1);

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

  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin1), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin1), updateRightEncoder, CHANGE);

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
    }else if (prefix == "for") {
      forwardSpeed = getNumber(btMessage, 1);
    } else if (prefix == "tim") {
      startMakerChecker = getNumber(btMessage, 1);
    } else if (prefix == "err") {
      marginError = getNumber(btMessage, 1);
    } else if (prefix == "pri") {
      printParameters();
    } else if (prefix == "sto") {
      stoppingPoint = getNumber(btMessage, 1);
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
    encoderValueLeft = 0;  // encoder goes to 0 after a run beggins
    encoderValueRight = 0;  // encoder goes to 0 after a run beggins
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

  // interrupts();

  vTaskResume(Task0);
  vTaskResume(Task1);
}

void loop()
{
	delay(100);
}
