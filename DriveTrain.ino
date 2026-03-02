

// FRONT RIGHT Wheel
int FR_IN1 = 12;  
int FR_IN2 = 13;
int FR_ENA = 11;   // 11 is PWM capable

// BACK LEFT Wheel
int BL_IN1 = 4;  
int BL_IN2 = 2;
int BL_ENA = 3;   // 3 is PWM capable

// FRONT LEFT Wheel 
int FL_IN1 = 7;  
int FL_IN2 = 6;
int FL_ENA = 5;   // 5 is PWM capable

// Back RIGHT Wheel
int BR_IN1 = 10;  
int BR_IN2 = 8;
int BR_ENA = 9;   // 9 is PWM capable


int FL_duty = 125; //
int FR_duty = 91; // done 7.24
int BL_duty = 95; //
int BR_duty = 105; // done

// int FL_duty = 125; //
// int FR_duty = 0; // done 7.24
// int BL_duty = 0; //
// int BR_duty = 0; // done
 
void setup()
{
  Serial.begin(9600);
  
  // Front Left Wheels
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FL_ENA, OUTPUT);

  digitalWrite(FL_IN1,LOW);
  digitalWrite(FL_IN2, LOW);

  // Front Right Wheel
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(FR_ENA, OUTPUT);

  digitalWrite(FR_IN1,LOW);
  digitalWrite(FR_IN2,LOW);

  // Back Left Wheel
  pinMode(BL_IN1, OUTPUT);
  pinMode(BL_IN2, OUTPUT);
  pinMode(BL_ENA, OUTPUT);

  digitalWrite(BL_IN1,LOW);
  digitalWrite(BL_IN2,LOW);

  // Back Right Wheel 
  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);
  pinMode(BR_ENA, OUTPUT);

  digitalWrite(BR_IN1,LOW);
  digitalWrite(BR_IN2,LOW);

   // PWM FOR THE FOUR MOTOR DRIVERS
 
  analogWrite(FL_ENA, FL_duty);
  analogWrite(FR_ENA, FR_duty);
  analogWrite(BL_ENA, BL_duty);
  analogWrite(BR_ENA, BR_duty);

  
}  
 
void loop() {

  // ALL POSSIBLE MOVEMENTS:
  //Forward();
  // Stop();
  //Forward();
  // Backward();
  Shift_Left();
  //Shift_Right();
  //Origin_Left();
  // Origin_Right();
  // Left_Front();
  // Right_Front();
  // Left_Rear();
  // Right_Rear();
  //Front_Axle_Left();
// Front_Axle_Right();
  // Rear_Axle_Left();
  // Rear_Axle_Right();
}

void turn_Right(){
    // Front Left Wheels
  CW(FL_IN1, FL_IN2);

  // Front Right Wheel
   CW(FR_IN1, FR_IN2);

  // Back Left Wheel
  CCW(BL_IN1, BL_IN2);

  // Back Right Wheel 
  CCW(BR_IN1, BR_IN2);
}

void turn_Left(){
    // Front Left Wheels
  CW(FL_IN1, FL_IN2);

  // Front Right Wheel
   CW(FR_IN1, FR_IN2);

  // Back Left Wheel
  CCW(BL_IN1, BL_IN2);

  // Back Right Wheel 
  CCW(BR_IN1, BR_IN2);
}


void Stop(void){

    // Front Left Wheels
  STOP(FL_IN1, FL_IN2);

  // Front Right Wheel
  STOP(FR_IN1, FR_IN2);

  // Back Left Wheel
  STOP(BL_IN1, BL_IN2);

  // Back Right Wheel 
  STOP(BR_IN1, BR_IN2);

}

void Shift_Right(void){

    // Front Left Wheels
  CW(FL_IN1, FL_IN2);

  // Front Right Wheel
   CW(FR_IN1, FR_IN2);

  // Back Left Wheel
   CW(BL_IN1, BL_IN2);

  // Back Right Wheel 
  CW(BR_IN1, BR_IN2);

}

void Shift_Left(void){

    // Front Left Wheels
  CCW(FL_IN1, FL_IN2);

  // Front Right Wheel
  CCW(FR_IN1, FR_IN2);

  // Back Left Wheel
  CCW(BL_IN1, BL_IN2);

  // Back Right Wheel 
  CCW(BR_IN1, BR_IN2);



}

void Backward(void){

    // Front Left Wheels

  CCW(FL_IN1, FL_IN2);

  // Front Right Wheel

  CW(FR_IN1, FR_IN2);

  // Back Left Wheel

  CW(BL_IN1, BL_IN2);

  // Back Right Wheel 

  CCW(BR_IN1, BR_IN2);

}

void Forward(void){

    // Front Left Wheels

  CW(FL_IN1, FL_IN2);

  // Front Right WheeL

  CCW(FR_IN1, FR_IN2);

  // Back Left Wheel

  CCW(BL_IN1, BL_IN2);

  // Back Right Wheel 

  CW(BR_IN1, BR_IN2);

}


void CW(int IN1 , int IN2){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

}

void CCW(int IN1 , int IN2){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

}

void STOP(int IN1, int IN2){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}


