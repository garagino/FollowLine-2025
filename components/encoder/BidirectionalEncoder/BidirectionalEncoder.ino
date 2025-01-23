#include <Arduino.h>

// Define encoder pins
int encoderLeftPin1 = 33;   // Encoder Output 'A' for left encoder
int encoderLeftPin2 = 32;   // Encoder Output 'B' for left encoder
int encoderRightPin1 = 25;  // Encoder Output 'A' for left encoder
int encoderRightPin2 = 26;  // Encoder Output 'B' for left encoder

volatile long encoderValueRight = 0;   // Variable to store encoder count
volatile long encoderValueLeft = 0;  // Variable to store encoder count

// Interrupt service routine for encoder A
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

void setup() {
  pinMode(encoderLeftPin1, INPUT);
  pinMode(encoderLeftPin2, INPUT);
  pinMode(encoderRightPin1, INPUT);
  pinMode(encoderRightPin2, INPUT);
  Serial.begin(115200);

  // Attach interrupt for encoder pin A (interrupt 0 is pin 2 on UNO)
  attachInterrupt(digitalPinToInterrupt(encoderLeftPin1), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderRightPin1), updateRightEncoder, CHANGE);
}

void loop() {
  // Set motor direction and speed
  // Do something with the encoder count
  // For example, stop the motor after a certain number of counts
  Serial.print("Esquerda: ");
  Serial.print(encoderValueLeft);
  Serial.print(" Direita: ");
  Serial.println(encoderValueRight);
}
