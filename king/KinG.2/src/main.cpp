/****************************************************************
 * Folow Line
 * CESAR School
 *
 * Line Follower PID with the microcontroller Vespa
 * from RoboCore and the Pololu's QTR-8RC sensor
 ****************************************************************/
// include
#include <QTRSensors.h> // Library for the QTR-8A or the QTR-8RC
#include "BluetoothSerial.h"

// Definições do BT
#define DEBUG
#define BT_NAME "kinG"

#ifdef DEBUG
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
// instancias
BluetoothSerial SerialBT; // Bluetooth Serial instance
QTRSensors qtr;           // QTR Sensor
#endif

// Definições de pinos
// Set button and led pins
const uint8_t PIN_BUTTON = 0;         // Botão
int estadoBotaoAnterior = HIGH;       // Estado anterior do botão
bool flagBotao = true;                // Flag do botão
unsigned long tempoUltimaLeitura = 0; // Tempo da última leitura
unsigned long DEBOUNCE_DELAY = 200;   // Tempo de debounce (50 ms)

// flag Sistema
bool flagLigarMotor = false; // Flag do botão

// flag Task
TaskHandle_t xHandleCalibragem = NULL;

// parametros
String btMessage;
String prefix;

const uint8_t PIN_MARKER_SENSOR = 36; // Sensor inicio/fim
// Motor control pins
#define INP1 27
#define INP2 14
#define ENA 13

#define INP3 25
#define INP4 26
#define ENB 12
//-------------------------------------------------]

void turnMotors(int leftSpeed, int rightSpeed);
void stopMotors();
String receiveBtMessage();
String getPrefix(String data);
double getNumber(String data, int index);
String getElement(String data, int index);
void printParameters();
int readSensors();
bool markerChecker();
//-------------------------------------------------

// Setup of the module of sensors
const uint8_t SENSOR_COUNT = 8;      // The number of sensors, which should match the length of the pins array
uint16_t sensorValues[SENSOR_COUNT]; // An array in which to store the calibrated sensor readings

// Maximum line position, considering the amount of sensors.
const long MAX_POSITION = (SENSOR_COUNT - 1) * 1000;

// Marker sensor variables
unsigned long startMakerChecker = 39500L;
unsigned long initialTime;

// Limit value of the margin of error
int marginError = 20;

bool firstRun = true;

//------------------PID Control-------------------
float p = 0, i = 0, d = 0, pid = 0, error = 0, lastError = 0;

float Kp = 0.3;
float Ki = 0.0001;
float Kd = 3.5;

int maxSpeed = 150;
int integralLimit = 200;
int lSpeed, rSpeed;

const bool LINE_BLACK = true;

bool limiter = false;

// Definições de tasks
void vTaskBotao(void *pvParameters)
{
  // Inicializa o pino botao
  pinMode(PIN_BUTTON, INPUT_PULLDOWN);
  for (;;)
  {
    int leituraAtual = digitalRead(PIN_BUTTON); // Lê o estado atual do botão

    // Verifica se já passou o tempo de debounce
    if (leituraAtual != estadoBotaoAnterior && (millis() - tempoUltimaLeitura) > DEBOUNCE_DELAY)
    {
      tempoUltimaLeitura = millis(); // Atualiza o tempo da última leitura

      // Quando o botão for pressionado (LOW)
      if (leituraAtual == LOW)
      {
        flagBotao = !flagBotao; // Alterna o estado da variável flagBotao
      }

      estadoBotaoAnterior = leituraAtual; // Atualiza o estado anterior
    }

    delay(10); // Pequeno delay para não sobrecarregar o processador
  }
}

// task calibração
void calibrationTask(void *pvParameters)
{
  bool flagLocal = false;
  while (1)
  {
    if (flagBotao == false && flagLocal == false)
    {
      qtr.calibrate();
      flagLocal = true;
    }
    else if (flagBotao == true && flagLocal == true)
    {
      vTaskDelay(2000);
      flagLigarMotor = true;
      flagLocal = false;
    }
    
    vTaskDelay(10);
  }
}

// Task mensager
void mensagerTask(void *pvParameters)
{
  for (;;)
  {
    if (flagBotao == false)
    {
      btMessage = receiveBtMessage();
      prefix = getPrefix(btMessage);

      if (prefix != "end")
      {
        if (prefix == "pid")
        {
          Kp = getNumber(btMessage, 1);
          Ki = getNumber(btMessage, 2);
          Kd = getNumber(btMessage, 3);
        }
        else if (prefix == "spe")
        {
          maxSpeed = getNumber(btMessage, 1);
        }
        else if (prefix == "tim")
        {
          startMakerChecker = getNumber(btMessage, 1);
        }
        else if (prefix == "err")
        {
          marginError = getNumber(btMessage, 1);
        }
        else if (prefix == "pri")
        {
          printParameters();
        }
        else if (prefix == "end")
        {
          break;
        }
        else if (prefix == "fro")
        {
          // Print the calibration minimum values measured when emitters were on
          for (uint8_t i = 0; i < SENSOR_COUNT; i++)
          {
            SerialBT.print(qtr.calibrationOn.minimum[i]);
            SerialBT.print(' ');
          }
          SerialBT.println();

          // Print the calibration maximum values measured when emitters were on
          for (uint8_t i = 0; i < SENSOR_COUNT; i++)
          {
            SerialBT.print(qtr.calibrationOn.maximum[i]);
            SerialBT.print(' ');
          }
          SerialBT.println();
        }
        else if (prefix == "lim")
        {
          if (limiter == false)
          {
            limiter = true;
            SerialBT.println("Limiter ativado!");
          }
          else
          {
            limiter = false;
            SerialBT.println("Limiter desativado!");
          }
        }
        else if (prefix == "f")
        {
          SerialBT.print(readSensors());
        }
        vTaskDelay(500);
      }
      vTaskDelay(500);
    }
    vTaskDelay(500);
  }
  vTaskDelay(10); // Pequeno delay para evitar watchdog
}

void setup()
{
  Serial.begin(115200);

  // Configura os pinos de controle como saída
  pinMode(INP1, OUTPUT);
  pinMode(INP2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(INP3, OUTPUT);
  pinMode(INP4, OUTPUT);
  pinMode(ENB, OUTPUT);

  analogWrite(INP1, 0);
  analogWrite(INP2, 0);
  analogWrite(INP3, 0);
  analogWrite(INP4, 0);

  // Configura os pinos dos sensores
  qtr.setTypeRC(); // For QTR-8RC      Sensor pins:
  qtr.setSensorPins((const uint8_t[]){16, 17, 5, 18, 19, 21, 22, 23}, SENSOR_COUNT);
  qtr.setTimeout(3000); // Ajusta o timeout para 3000 µs

  // Configura do sensor de inicio/fim
  pinMode(PIN_MARKER_SENSOR, INPUT);

#ifdef DEBUG
  if (firstRun)
  {
    Serial.begin(115200);
    delay(100);

    SerialBT.begin(BT_NAME); // Bluetooth device name
    firstRun = false;
  }
  SerialBT.println("Start BT communication");

  delay(500);
#endif

  // Criar a task do calibrador
  xTaskCreate(
      vTaskBotao,   // Função da task
      "vTaskBotao", // Nome da task
      2048,         // Tamanho da stack (em palavras)
      NULL,         // Parâmetro da task
      2,            // Prioridade
      NULL          // Handle da task (opcional)
  );

  // Criar a task do calibrador
  xTaskCreate(
      mensagerTask,   // Função da task
      "mensagerTask", // Nome da task
      2048,           // Tamanho da stack (em palavras)
      NULL,           // Parâmetro da task
      4,              // Prioridade
      NULL            // Handle da task (opcional)
  );

  // Criar a task do calibrador
  xTaskCreate(
      calibrationTask,   // Função da task
      "calibrationTask", // Nome da task
      2048,              // Tamanho da stack (em palavras)
      NULL,              // Parâmetro da task
      1,                 // Prioridade
      &xHandleCalibragem // Handle da task (opcional)
  );
  initialTime = millis();
}

void loop()
{
  if (flagLigarMotor == true && flagBotao == true)
  {
    // readSensors() returns the line position between 0 and `MAX_POSITION`.
    // error is a re-map from -1000 to 1000 range.
    error = map(readSensors(), 0, MAX_POSITION, -1000, 1000);

    uint16_t position = qtr.readLineWhite(sensorValues);

    //ts print
    for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    {
      Serial.printf("Sensor %d: %d\t", i, sensorValues[i]);
    }
    
    // Calculate PID
    p = error;
    i = i + error;
    if (limiter == true)
    {
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

    if (markerChecker())
    { // Count the markers and stop the robot when reach a certain number
      stopMotors();
    }
    else if (error >= -marginError && error <= marginError)
    { // If the error is within the MARGIN_ERROR, move on
      turnMotors(maxSpeed, maxSpeed);
    }
    else
    { // If the error is outside the error range, continue doing PID
      turnMotors(lSpeed, rSpeed);
    }
  }
  else if (flagLigarMotor == true && flagBotao == false)
  {
    stopMotors();
    flagLigarMotor = false;
  }
}

int readSensors()
{
  if (LINE_BLACK)
  {
    return qtr.readLineBlack(sensorValues);
  }
  else
  {
    return qtr.readLineWhite(sensorValues);
  }
}

/**
  Verifies if there is a end line after a set time

  @return `true` if the end line was detected.
*/
bool markerChecker()
{
  static int i = 0;
  if (startMakerChecker < millis() - initialTime)
  {
    if (analogRead(PIN_MARKER_SENSOR) < 2000)
    {
      return true;
    }
  }

  return false;
}

#ifdef DEBUG

/**
  Returns all stream of data sent over bluetooth until the
  button is pressed.

  @return `String` with the message sent by the bluetooth device
*/
String receiveBtMessage()
{
  String message;
  char incomingChar;

  while (flagBotao == false)
  {
    if (SerialBT.available())
    {
      incomingChar = SerialBT.read();

      if (incomingChar == '\n')
        break;

      message += String(incomingChar);
    }
  }

  message.trim();
  return message;
}

String getPrefix(String data)
{
  return getElement(data, 0);
}

double getNumber(String data, int index)
{
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
String getElement(String data, int index)
{
  char separator = ' ';
  int found = 0;
  int startIndex = 0, endIndex = -1;
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data.charAt(i) == separator || i == maxIndex)
    {
      found++;
      startIndex = endIndex + 1;
      endIndex = (i == maxIndex) ? i + 1 : i;
    }
  }

  if (found <= index)
  {
    return "";
  }

  return data.substring(startIndex, endIndex);
}

void printParameters()
{
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
}

void turnMotors(int leftSpeed, int rightSpeed)
{
  if (leftSpeed >= 0)
  {
    analogWrite(INP1, leftSpeed);
    analogWrite(INP2, 0);
  }
  else
  {
    analogWrite(INP1, 0);
    analogWrite(INP2, -leftSpeed);
  }

  if (rightSpeed >= 0)
  {
    analogWrite(INP3, rightSpeed);
    analogWrite(INP4, 0);
  }
  else
  {
    analogWrite(INP3, 0);
    analogWrite(INP4, -rightSpeed);
  }
}

void stopMotors()
{
  analogWrite(INP1, 0);
  analogWrite(INP2, 0);
  analogWrite(INP3, 0);
  analogWrite(INP4, 0);
}

#endif