/*
    detects far beacon using absolute voltage level 
    - near beacon: ~5.0 V at the sensor
    - far beacon:  ~2.2 V at the sensor
    - Nose / ambient: a few hundred mV
*/

#include <MecanumWheels.h>

// Pin definitions (IN1, IN2, ENA for each wheel: FL, FR, BL, BR)
MecanumWheels robot(
    26, 27, 3,    // Front Left:  IN1, IN2, ENA
    22, 23, 11, // Front Right: IN1, IN2, ENA
    28, 29, 2,    // Back Left:   IN1, IN2, ENA
    24, 25, 10    // Back Right:  IN1, IN2, ENA
);

const int sensorPin = A0;        //  analog input from the Schmitt trigger

const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz)

// voltage bounds 
const float FAR_MIN_V  = 1.5;    // lower bound around the ~2.2 V far beacon
const float FAR_MAX_V  = 3.0;    // upper bound around the ~2.2 V far beacon
const float NEAR_MIN_V = 4.0;    // lower bound around the ~5.0 V near beacon
const float NEAR_MAX_V = 5.2;    // upper bound around the ~5.0 V near beacon

volatile bool facingFarBeacon  = false;   // true when aligned with far beacon
volatile bool facingNearBeacon = false;   // true when aligned with near beacon

volatile bool finishedRotating = false;

double voltage = 0.0;
const float referenceVoltage = 2.5;

// Threshold Setting
const float thresholdAmplitude = 3.8/2; // Trigger if wave reaches > 1.0V away from 2.5V
bool isThresholdExceeded = false;


void setup() {
    Serial.begin(9600);
    // begin() sets up pins and applies default duty cycles
    // You can override: robot.begin(fl, fr, bl, br)
    robot.begin(125, 91, 95, 105);
    //robot.turnRight();
    robot.turnLeft();
}

void loop() {
  unsigned long startMillis = millis();
  float maxDisplacement = 0.0;

  // 1. Sample the wave for 20ms
  if (millis() - startMillis < sampleWindow) {
    int rawADC = analogRead(sensorPin);
    float currentVoltage = (rawADC * 5.0) / 1023.0;
    float displacement = abs(currentVoltage - referenceVoltage);
    
    if (displacement > maxDisplacement) {
      maxDisplacement = displacement;
    }
    
  }

  // 2. Threshold Check
  if (maxDisplacement > thresholdAmplitude) {
    isThresholdExceeded = true;
    robot.stop();
  } else {
    isThresholdExceeded = false;
  }

}



