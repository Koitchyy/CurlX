#include <MecanumWheels.h>

// Pins mapped as: (IN1, IN2, PWM)
MecanumWheels robot(
    27, 26, 3,   // FL: Front Left
    22, 23, 11,  // FR: Front Right
    29, 28, 2,   // BL: Back Left
    24, 25, 10   // BR: Back Right
);

// --- Top-Level Configuration ---
const int sensorPin = A0;
const int flywheelPin = 4;
const int dirPin = 6;
const int stepPin = 7;
const float thresholdAmplitude = 1.15; 

// Timing Configuration
const unsigned long shiftDuration = 2000; 
const unsigned long reverseDuration = 3000;
const int shootSpeed = 130;

// Stepper Configuration
const int stepsTo90 = 888;          
const unsigned long stepDelayMicros = 1000; 
const unsigned long stepInterval = 10000; // 5 second pause between shots
const unsigned long finalWait = 1500; 

// --- State Management ---
enum State { SEARCHING, SHIFTING_RIGHT, REVERSING, SHOOTING, FINISHED };
State currentState = SHOOTING; // Set to SEARCHING for full robot run

enum ShootSubState { WAIT1, TURN1, WAIT2, TURN2, WAIT3, TURN3, FINAL_DELAY };
ShootSubState shootPhase = WAIT1;

unsigned long stateStartTime = 0;
unsigned long shootPhaseStartTime = 0;

// --- Stepper Control Variables ---
unsigned long lastStepMicros = 0;
int currentStepCount = 0;
bool stepPinState = LOW;
bool stepperActive = false;

void setup() {
    Serial.begin(9600);
    pinMode(flywheelPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    
    digitalWrite(dirPin, HIGH); 
    
    // Motor speed offsets: FL, FR, BL, BR
    robot.begin(115, 100, 100, 100);
    robot.stop(); 

    // Timer initialization for testing SHOOTING state
    shootPhaseStartTime = millis(); 
    analogWrite(flywheelPin, shootSpeed);
    Serial.println("Code Loaded. FL(27,26,3) FR(22,23,11) BL(29,28,2) BR(24,25,10)");
}

void loop() {
    runStepperNonBlocking();

    switch (currentState) {
        case SEARCHING:
            if (getMaxDisplacement() > thresholdAmplitude) {
                robot.stop();
                startNewState(SHIFTING_RIGHT);
                robot.shiftRight();
            }
            break;
        case SHIFTING_RIGHT:
            if (isTimeElapsed(stateStartTime, shiftDuration)) {
                robot.stop();
                startNewState(REVERSING);
                robot.backward();
            }
            break;
        case REVERSING:
            if (isTimeElapsed(stateStartTime, reverseDuration)) {
                robot.stop();
                startNewState(SHOOTING);
                shootPhase = WAIT1;
                shootPhaseStartTime = millis();
                analogWrite(flywheelPin, shootSpeed); 
            }
            break;
        case SHOOTING:
            handleShootingSequence();
            break;
        case FINISHED:
            break;
    }
}

// --- Stepper Engine (No Acceleration) ---
void runStepperNonBlocking() {
    if (!stepperActive) return;

    if (micros() - lastStepMicros >= stepDelayMicros) {
        lastStepMicros = micros();
        stepPinState = !stepPinState;
        digitalWrite(stepPin, stepPinState);

        if (stepPinState == LOW) {
            currentStepCount++;
            if (currentStepCount >= stepsTo90) {
                stepperActive = false;
                digitalWrite(stepPin, LOW);
                Serial.println("Rotation Complete.");
            }
        }
    }
}

void request90DegreeTurn() {
    currentStepCount = 0;
    lastStepMicros = micros();
    stepperActive = true;
}

// --- Shooting Sequence Manager ---
void handleShootingSequence() {
    switch (shootPhase) {
        case WAIT1:
            if (isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                Serial.println("Turn 1 start");
                request90DegreeTurn();
                advanceShootPhase(TURN1);
            }
            break;
        case TURN1:
            if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                Serial.println("Turn 2 start");
                request90DegreeTurn();
                advanceShootPhase(TURN2);
            }
            break;
        case TURN2:
            if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                Serial.println("Turn 3 start");
                request90DegreeTurn();
                advanceShootPhase(TURN3);
            }
            break;
        case TURN3:
            if (!stepperActive) {
                advanceShootPhase(FINAL_DELAY);
            }
            break;
        case FINAL_DELAY:
            if (isTimeElapsed(shootPhaseStartTime, finalWait)) {
                analogWrite(flywheelPin, 0); 
                currentState = FINISHED;
                Serial.println("All sequences complete.");
            }
            break;
    }
}

void startNewState(State newState) {
    stateStartTime = millis();
    currentState = newState;
}

void advanceShootPhase(ShootSubState nextPhase) {
    shootPhaseStartTime = millis(); // Reset timer for the next 5s wait
    shootPhase = nextPhase;
}

float getMaxDisplacement() {
    float maxDisplacement = 0.0;
    unsigned long startMillis = millis();
    while (millis() - startMillis < 50) {
        float v = (analogRead(sensorPin) * 5.0) / 1023.0;
        float d = abs(v - 2.5);
        if (d > maxDisplacement) maxDisplacement = d;
    }
    return maxDisplacement;
}

bool isTimeElapsed(unsigned long startTime, unsigned long duration) {
    return (millis() - startTime >= duration);
}