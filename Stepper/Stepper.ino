// Pin Definitions
const int stepPin = 3;  // Connected to STEP on DRV8825
const int dirPin = 2;   // Connected to DIR on DRV8825

// Constants for 90 degrees
// Formula: (90 / 1.8) = 50 steps
const int stepsPer45 = 50*16 ;

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  
  // Set direction (HIGH or LOW)
  digitalWrite(dirPin, HIGH); 
  
  delay(1000); // Wait a second before starting
  
  move45Degrees();
}

void loop() {
  // digitalWrite(stepPin, HIGH);
  // delayMicroseconds(10000); // Increase this number to go slower
  // digitalWrite(stepPin, LOW);
  // delayMicroseconds(10000); 
}

void move45Degrees() {
  for(int x = 0; x < stepsPer45; x++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(625); // Increase this number to go slower
    digitalWrite(stepPin, LOW);
    delayMicroseconds(625); 
  }
}