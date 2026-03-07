#include <MecanumWheels.h>

// Pins mapped as: (IN1, IN2, PWM)
MecanumWheels robot(
    27, 26, 3,   // FL
    22, 23, 11,  // FR
    29, 28, 2,   // BL
    24, 25, 10   // BR
);

// --- Pin Definitions ---
const int beaconPin = A0;   
const int flywheelPin = 4;
const int dirPin = 6;
const int stepPin = 7;
const int lineLedPin = 49;  
const int lineSensorPin = 18; 

// --- Thresholds and Constants ---
const float thresholdAmplitude = 2.1; 
const int lineThreshold = 50;         
const unsigned long lineSampleWindow = 100;
const unsigned long rightDuration = 500;
// Timing Configuration
const unsigned long reverseDuration = 4000;
const int shootSpeed = 130;

// Stepper Configuration
const int stepsTo90 = 888;          
const unsigned long stepDelayMicros = 1000; 
const unsigned long stepInterval = 8000; 
const unsigned long finalWait = 1500; 

// --- Variables for Line Sensor ---
volatile unsigned int pulseCount = 0;
unsigned int lastSavedPulseCount = 0;
unsigned long lastLineCheckTime = 0;

// --- State Management ---
enum State { SEARCHING, SHIFTING_RIGHT, SHIFTING_MORE_RIGHT, REVERSING, SHOOTING, FINISHED };
State currentState = SEARCHING;

enum ShootSubState { WAIT1, TURN1, WAIT2, TURN2, WAIT3, TURN3, FINAL_DELAY };
ShootSubState shootPhase = WAIT1;

unsigned long stateStartTime = 0;
unsigned long shootPhaseStartTime = 0;

// --- Stepper Control Variables ---
unsigned long lastStepMicros = 0;
int currentStepCount = 0;
bool stepPinState = LOW;
bool stepperActive = false;

void countPulse() { pulseCount++; }

void setup() {
    Serial.begin(9600);
    
    pinMode(flywheelPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(lineLedPin, OUTPUT);
    pinMode(lineSensorPin, INPUT);
    
    digitalWrite(dirPin, HIGH); 

    tone(lineLedPin, 1250);
    attachInterrupt(digitalPinToInterrupt(lineSensorPin), countPulse, FALLING);
    
    // Default High Speed for Search
    robot.begin(115, 100, 100, 100);
    
    Serial.println("System Initialized. Starting Search...");
    robot.turnRight();
    stateStartTime = millis();
}

void loop() {
    runStepperNonBlocking();
    updateLineSensor(); 

    switch (currentState) {
        case SEARCHING:
            if (getMaxDisplacement() > thresholdAmplitude) {
                robot.stop();
                Serial.println("Beacon Found. Shifting to Line (Low Speed)...");
                startNewState(SHIFTING_RIGHT);
                robot.shiftRight();
            }
            break;

        case SHIFTING_RIGHT:
            if (isLineDetected()) {
                robot.stop();
                Serial.println("Line Detected! Reversing (High Speed)...");
                startNewState(SHIFTING_MORE_RIGHT);
                robot.shiftRight();
            }
            break;

        case SHIFTING_MORE_RIGHT:
            if (isTimeElapsed(stateStartTime, rightDuration)) {
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
                Serial.println("In Position. Flywheel ON.");
            }
            break;

        case SHOOTING:
            handleShootingSequence();
            break;

        case FINISHED:
            break;
    }
}

// --- Dynamic Speed Control ---
void startNewState(State newState) {
    stateStartTime = millis();
    currentState = newState;

    if (newState == SHIFTING_RIGHT || newState == SHIFTING_MORE_RIGHT) {
        // Lower PWM for precise line detection
        robot.begin(78, 71, 88, 95);
        
        // Reset line sensor data to avoid immediate false triggers
        noInterrupts();
        pulseCount = 0;
        lastSavedPulseCount = 125; 
        interrupts();
        lastLineCheckTime = millis();
    } 
    else {
        // Higher PWM for Search and Reverse
        robot.begin(115, 100, 100, 100);
    }
}

// --- Helper Functions (Line, Stepper, Timing) ---

void updateLineSensor() {
    unsigned long currentTime = millis();
    if (currentTime - lastLineCheckTime >= lineSampleWindow) {
        noInterrupts();
        lastSavedPulseCount = pulseCount;
        pulseCount = 0;
        interrupts();
        lastLineCheckTime = currentTime;
    }
}

bool isLineDetected() {
    return (lastSavedPulseCount < lineThreshold);
}

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
            }
        }
    }
}

void request90DegreeTurn() {
    currentStepCount = 0;
    lastStepMicros = micros();
    stepperActive = true;
}

void handleShootingSequence() {
    switch (shootPhase) {
        case WAIT1:
            if (isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                request90DegreeTurn();
                advanceShootPhase(TURN1);
            }
            break;
        case TURN1:
            if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                request90DegreeTurn();
                advanceShootPhase(TURN2);
            }
            break;
        case TURN2:
            if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) {
                request90DegreeTurn();
                advanceShootPhase(TURN3);
            }
            break;
        case TURN3:
            if (!stepperActive) advanceShootPhase(FINAL_DELAY);
            break;
        case FINAL_DELAY:
            if (isTimeElapsed(shootPhaseStartTime, finalWait)) {
                analogWrite(flywheelPin, 0); 
                currentState = FINISHED;
                Serial.println("Mission Complete.");
            }
            break;
    }
}

void advanceShootPhase(ShootSubState nextPhase) {
    shootPhaseStartTime = millis();
    shootPhase = nextPhase;
}

float getMaxDisplacement() {
    float maxDisplacement = 0.0;
    unsigned long startMillis = millis();
    while (millis() - startMillis < 50) {
        float v = (analogRead(beaconPin) * 5.0) / 1023.0;
        float d = abs(v - 2.5);
        if (d > maxDisplacement) maxDisplacement = d;
    }
    return maxDisplacement;
}

bool isTimeElapsed(unsigned long startTime, unsigned long duration) {
    return (millis() - startTime >= duration);
}