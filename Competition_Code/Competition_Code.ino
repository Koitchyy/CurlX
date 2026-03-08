#include <MecanumWheels.h>

// --- Global Control Variables ---
unsigned long globalStartTime = 0;
const unsigned long MAX_RUNTIME_MS = 120000; // 2 Minutes
int cycleCount = 0;
const int MAX_CYCLES = 3;

// Pins
MecanumWheels robot(27, 26, 3, 22, 23, 11, 29, 28, 2, 24, 25, 10);

const int beaconPin = A0;   
const int flywheelPin = 4;
const int dirPin = 6;
const int stepPin = 7;
const int lineLedPin = 49;  
const int lineSensorPin = 18; 

// Thresholds and Constants
const float thresholdAmplitude = 2.1; 
const int lineThreshold = 50;         
const unsigned long lineSampleWindow = 100;
const unsigned long rightDuration = 950;
const unsigned long fastRightTime = 100;
const unsigned long returnDuration = 2000; 
const unsigned long reverseDuration = 2000;
const unsigned long reverseAgainDuration = 500;
const unsigned long reloadDuration = 5000; // 5 Seconds to reload
int shootSpeed = 145;
const int stepsTo90 = 825;           
const unsigned long stepDelayMicros = 1000; 
const unsigned long stepInterval = 9000; 
const unsigned long finalWait = 1500; 

// State Management
enum State { SEARCHING, REVERSING, FAST_SHIFTING_RIGHT, SHIFTING_RIGHT, SHIFTING_MORE_RIGHT, REVERSING_AGAIN, SHOOTING, RETURNING, RELOAD, FINISHED };
State currentState = SEARCHING;

enum ShootSubState { WAIT1, TURN1, WAIT2, TURN2, WAIT3, TURN3, FINAL_DELAY };
ShootSubState shootPhase = WAIT1;

unsigned long stateStartTime = 0;
unsigned long shootPhaseStartTime = 0;
volatile unsigned int pulseCount = 0;
unsigned int lastSavedPulseCount = 0;
unsigned long lastLineCheckTime = 0;
unsigned long lastStepMicros = 0;
int currentStepCount = 0;
bool stepPinState = LOW;
bool stepperActive = false;

void countPulse() { pulseCount++; }

void setup() {
    Serial.begin(9600);
    pinMode(flywheelPin, OUTPUT); pinMode(dirPin, OUTPUT);
    pinMode(stepPin, OUTPUT); pinMode(lineLedPin, OUTPUT);
    pinMode(lineSensorPin, INPUT);
    digitalWrite(dirPin, HIGH); 
    tone(lineLedPin, 1250);
    attachInterrupt(digitalPinToInterrupt(lineSensorPin), countPulse, FALLING);
    
    globalStartTime = millis(); 
    robot.begin(115, 100, 100, 100);
    Serial.println("System Initialized. Starting Run 1...");
    robot.turnRight();
    //robot.shiftRight();
    stateStartTime = millis();
}

void loop() {
    // --- 2-Minute Master Kill Switch ---
    if (millis() - globalStartTime >= MAX_RUNTIME_MS) {
        if (currentState != FINISHED) {
            robot.stop();
            analogWrite(flywheelPin, 0);
            currentState = FINISHED;
            Serial.println("!!! TIME LIMIT REACHED. SHUTTING DOWN !!!");
        }
        return;
    }

    runStepperNonBlocking();
    updateLineSensor(); 

    switch (currentState) {
        case SEARCHING:
            if (getMaxDisplacement() > thresholdAmplitude) {
                robot.stop();
                startNewState(REVERSING);
                robot.backward();
            }
            break;

        case REVERSING:
            if (isTimeElapsed(stateStartTime, reverseDuration)) {
                robot.stop();
                startNewState(FAST_SHIFTING_RIGHT);
                robot.shiftRight();
            }
            break;
        
         case FAST_SHIFTING_RIGHT:
            if (isTimeElapsed(stateStartTime, fastRightTime)) {
                //robot.stop();
                startNewState(SHIFTING_RIGHT);
                robot.shiftRight();
            }
            break;

        case SHIFTING_RIGHT:
            if (isLineDetected()) {
                //robot.stop();
                startNewState(SHIFTING_MORE_RIGHT);
                robot.shiftRight();
            }
            break;

        case SHIFTING_MORE_RIGHT:
            if (isTimeElapsed(stateStartTime, rightDuration)) {
                robot.stop();
                startNewState(REVERSING_AGAIN);
                robot.backward(); 
            }
            break;

        case REVERSING_AGAIN:
            if (isTimeElapsed(stateStartTime, reverseAgainDuration)) {
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

        case RETURNING:
            if (isTimeElapsed(stateStartTime, returnDuration)) {
                robot.stop();
                Serial.println("Returned. Reloading 5s...");
                startNewState(RELOAD);
            }
            break;


        case RELOAD:
            if (isTimeElapsed(stateStartTime, reloadDuration)) {
                cycleCount++;
                if (cycleCount < MAX_CYCLES) {
                    Serial.print("Reload done. Starting Run "); Serial.println(cycleCount + 1);
                    if (cycleCount == 1 || cycleCount == 2){
                        shootSpeed = 138;
                    }
                    startNewState(FAST_SHIFTING_RIGHT);
                    robot.shiftRight();
                } 
                else {
                    Serial.println("All cycles complete.");
                    startNewState(FINISHED);
                }
            }
            break;

        case FINISHED:
            robot.stop();
            analogWrite(flywheelPin, 0);
            break;
    }
}

void startNewState(State newState) {
    stateStartTime = millis();
    currentState = newState;
    if (newState == SHIFTING_RIGHT || newState == SHIFTING_MORE_RIGHT) {
        robot.begin(72, 72, 81, 103);
        noInterrupts(); pulseCount = 0; lastSavedPulseCount = 125; interrupts();
        lastLineCheckTime = millis();
    } else if(newState == FAST_SHIFTING_RIGHT) {
        robot.begin(80, 80, 90, 115);
    } 
    else if(newState == REVERSING || newState == REVERSING_AGAIN){
        robot.begin(115, 100, 100, 115);
    }
    else {
        robot.begin(115, 100, 100, 100);
    }
}

// --- Helper Functions ---
void updateLineSensor() {
    unsigned long currentTime = millis();
    if (currentTime - lastLineCheckTime >= lineSampleWindow) {
        noInterrupts(); lastSavedPulseCount = pulseCount; pulseCount = 0; interrupts();
        lastLineCheckTime = currentTime;
    }
}

bool isLineDetected() { return (lastSavedPulseCount < lineThreshold); }

void runStepperNonBlocking() {
    if (!stepperActive) return;
    if (micros() - lastStepMicros >= stepDelayMicros) {
        lastStepMicros = micros();
        stepPinState = !stepPinState;
        digitalWrite(stepPin, stepPinState);
        if (stepPinState == LOW) {
            currentStepCount++;
            if (currentStepCount >= stepsTo90) { stepperActive = false; digitalWrite(stepPin, LOW); }
        }
    }
}

void request90DegreeTurn() { currentStepCount = 0; lastStepMicros = micros(); stepperActive = true; }

void handleShootingSequence() {
    switch (shootPhase) {
        case WAIT1: if (isTimeElapsed(shootPhaseStartTime, stepInterval)) { request90DegreeTurn(); advanceShootPhase(TURN1); } break;
        case TURN1: if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) { request90DegreeTurn(); advanceShootPhase(TURN2); } break;
        case TURN2: if (!stepperActive && isTimeElapsed(shootPhaseStartTime, stepInterval)) { request90DegreeTurn(); advanceShootPhase(TURN3); } break;
        case TURN3: if (!stepperActive) advanceShootPhase(FINAL_DELAY); break;
        case FINAL_DELAY:
            if (isTimeElapsed(shootPhaseStartTime, finalWait)) {
                analogWrite(flywheelPin, 0); 
                startNewState(RETURNING);
                robot.shiftLeft(); 
            }
            break;
    }
}

void advanceShootPhase(ShootSubState nextPhase) { shootPhaseStartTime = millis(); shootPhase = nextPhase; }

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

bool isTimeElapsed(unsigned long startTime, unsigned long duration) { return (millis() - startTime >= duration); }
