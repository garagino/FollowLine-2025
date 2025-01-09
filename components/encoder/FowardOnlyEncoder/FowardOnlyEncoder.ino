


// Encoder ports at Robocore's Vespa
int encoderLeftPin1 = 25;
int encoderLeftPin2 = 26; 

int encoderRightPin1 = 33; 
int encoderRightPin2 = 32; 

volatile long encoderValue = 0; // Raw encoder ticks count

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
    
  Serial.print("Encoder: " );
  Serial.println(encoderValue);
    
}

void updateEncoder(){

  encoderValue ++;

}