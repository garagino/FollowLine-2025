


// Encoder ports at Robocore's Vespa
int encoderLeftPin1 = 25;
int encoderLeftPin2 = 26; 

int encoderRightPin1 = 33; 
int encoderRightPin2 = 32; 


volatile int lastEncodedLeft = 0; // Here updated value of encoder store.
volatile int lastEncodedRight = 0; // Here updated value of encoder store.

volatile long encoderLeftValue = 0; // Raw encoder value
volatile long encoderRightValue = 0; // Raw encoder value


void setup(){


  // Sets serial speed
  Serial.begin(115200);

  // Configures All encoders to PullUp 
  pinMode(encoderLeftPin1, INPUT_PULLUP); 
  pinMode(encoderLeftPin2, INPUT_PULLUP);

  pinMode(encoderRightPin1, INPUT_PULLUP); 
  pinMode(encoderRightPin2, INPUT_PULLUP);


  //turn pullup resistor on
  digitalWrite(encoderLeftPin1, HIGH);
  digitalWrite(encoderLeftPin2, HIGH);

  digitalWrite(encoderRightPin1, HIGH);
  digitalWrite(encoderRightPin2, HIGH);



  //call updateEncoder() when any high/low changed seen

  attachInterrupt(encoderLeftPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderRightPin1, updateEncoder, CHANGE);
  attachInterrupt(encoderLeftPin2, updateEncoder, CHANGE);
  attachInterrupt(encoderRightPin2, updateEncoder, CHANGE);

}

void loop(){
    
  Serial.print("Esquerda: " );
  Serial.println(encoderLeftValue);
  Serial.print("Direita: " );
  Serial.println(encoderRightValue);
    
}

void updateEncoder(){


  int MSB = digitalRead(encoderLeftPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderLeftPin2); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting thex 2 pin value to single number
  int sum  = (lastEncodedLeft << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderLeftValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderLeftValue ++;

  lastEncodedLeft = encoded; //store this value for next time

  MSB = digitalRead(encoderRightPin1); //MSB = most significant bit
  LSB = digitalRead(encoderRightPin2); //LSB = least significant bit

  encoded = (MSB << 1) |LSB; //converting thex 2 pin value to single number
  sum  = (lastEncodedRight << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoderRightValue --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoderRightValue ++;

  lastEncodedRight = encoded; //store this value for next time

}