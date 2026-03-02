// --- Pin Definitions ---
const int ledPin = 9;    // Outputs the 1.25 kHz signal to the LED
const int sensorPin = 2; // Reads the output from the Schmitt Trigger

// --- Variables for Counting ---
volatile unsigned int pulseCount = 0;
unsigned int savedCount = 0;
unsigned long lastTime = 0;

const int sampleWindow = 100; // ms

void setup() {
  Serial.begin(9600);

  pinMode(ledPin, OUTPUT);
  pinMode(sensorPin, INPUT);

  // Continuous 1.25 kHz square wave on ledPin
  tone(ledPin, 1250);

  // Attach an interrupt to Pin 2.
  // counts falling edges
  attachInterrupt(digitalPinToInterrupt(sensorPin), countPulse, FALLING);
  Serial.println("Modulated IR Sensor Test Started...");
}

void loop() {
  unsigned long currentTime = millis();

  // Check if our 100ms sample window has passed
  if (currentTime - lastTime >= sampleWindow) {

    // 1. Pause interrupts temporarily to safely read and clear the counter
    noInterrupts();
    savedCount = pulseCount;
    pulseCount = 0;
    interrupts();

    // 2. Analyze the data
    // At 1.25 kHz, we expect ~1250 pulses per second, or ~125 pulses per 100ms.
    // If it's looking at black tape, the pulses will stop (count drops near 0).

    Serial.print("Pulses read: ");
    Serial.print(savedCount);

    // We use a threshold (e.g., 50) to act as a buffer against noise
    if (savedCount > 50) {
      Serial.println("   |   Status: WHITE (Reflecting)");
    } else {
      Serial.println("   |   Status: BLACK (Line Detected!)");
    }

    lastTime = currentTime;
  }
}

// --- Interrupt Service Routine (ISR) ---
// This runs automatically in the background every time Pin 2 detects a falling
// edge
void countPulse() { pulseCount++; }