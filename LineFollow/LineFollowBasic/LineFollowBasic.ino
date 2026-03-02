// --- Pin Definitions ---
const int irLedPin = 9; // ONE pin powers all 3 IR LEDs

// MUST be interrupt-capable pins (e.g., Pins 2, 3, 18, 19, 20, 21 on Arduino
// Mega)
const int sensorLeadPin = 2;
const int sensorLeftPin = 3;
const int sensorRightPin = 18;

// --- Volatile Counters for ISRs ---
volatile unsigned int pulseLead = 0;
volatile unsigned int pulseLeft = 0;
volatile unsigned int pulseRight = 0;

// --- Timing and Logic Variables ---
unsigned long lastTime = 0;
const int sampleWindow = 100; // ms
const int noiseThreshold = 50;
const int freq = 1250;

// State Management
enum DriveState {
  FORWARD,
  SLIGHT_LEFT,
  HARD_LEFT,
  SLIGHT_RIGHT,
  HARD_RIGHT,
  LOST
};

void setup() {
  Serial.begin(9600);

  // Set up pins
  pinMode(irLedPin, OUTPUT);
  pinMode(sensorLeadPin, INPUT);
  pinMode(sensorLeftPin, INPUT);
  pinMode(sensorRightPin, INPUT);

  // Continuous 1.25 kHz square wave on ledPin
  tone(irLedPin, freq);

  attachInterrupt(digitalPinToInterrupt(sensorLeadPin), countLead, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorLeftPin), countLeft, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorRightPin), countRight, FALLING);

  Serial.println("Modulated IR Follower Started...");
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= sampleWindow) {
    // grab all pulse counts
    noInterrupts();
    unsigned int pLead = pulseLead;
    pulseLead = 0;
    unsigned int pLeft = pulseLeft;
    pulseLeft = 0;
    unsigned int pRight = pulseRight;
    pulseRight = 0;
    interrupts();

    // 2. Convert raw pulse counts to binary line states
    // If pulses < threshold, the IR is absorbed by black tape (Line = 1)
    // If pulses > threshold, IR is reflecting off white floor (No Line = 0)
    bool s1 = (pLead < noiseThreshold);
    bool s2 = (pLeft < noiseThreshold);
    bool s3 = (pRight < noiseThreshold);

    // 3. Map the sensor states to a driving action
    DriveState state = determineState(s1, s2, s3);

    // 4. Execute the State (Replacing with serial prints for now)
    executeState(state);

    lastTime = currentTime;
  }
}

// --- Helper Functions ---

DriveState determineState(bool lead, bool left, bool right) {
  if (lead && !left && !right)
    return FORWARD; // 1 0 0
  if (lead && left && !right)
    return SLIGHT_LEFT; // 1 1 0
  if (!lead && left && !right)
    return HARD_LEFT; // 0 1 0
  if (lead && !left && right)
    return SLIGHT_RIGHT; // 1 0 1
  if (!lead && !left && right)
    return HARD_RIGHT; // 0 0 1
  return LOST;         // 0 0 0 or anything else
}

void executeState(DriveState state) {
  switch (state) {
  case FORWARD:
    Serial.println("FWD - Full Speed Ahead");
    break;
  case SLIGHT_LEFT:
    Serial.println("SLIGHT LEFT - Right motors spin FWD faster");
    break;
  case HARD_LEFT:
    Serial.println("HARD LEFT - Spin ONLY right motors");
    break;
  case SLIGHT_RIGHT:
    Serial.println("SLIGHT RIGHT - Left motors spin FWD faster");
    break;
  case HARD_RIGHT:
    Serial.println("HARD RIGHT - Spin ONLY left motors");
    break;
  case LOST:
    Serial.println("LOST - Beacon Sensing (head in beacon direction)");
    break;
  }
}

// --- Interrupt Service Routines (ISRs) ---
void countLead() { pulseLead++; }
void countLeft() { pulseLeft++; }
void countRight() { pulseRight++; }