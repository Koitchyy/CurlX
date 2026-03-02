/*
    detects far beacon using absolute voltage level 
    - near beacon: ~5.0 V at the sensor
    - far beacon:  ~2.2 V at the sensor
    - Nose / ambient: a few hundred mV
*/

const int sensorPin = A1;        //  analog input from the Schmitt trigger


// voltage bounds 
const float FAR_MIN_V  = 1.5;    // lower bound around the ~2.2 V far beacon
const float FAR_MAX_V  = 3.0;    // upper bound around the ~2.2 V far beacon
const float NEAR_MIN_V = 4.0;    // lower bound around the ~5.0 V near beacon
const float NEAR_MAX_V = 5.2;    // upper bound around the ~5.0 V near beacon

volatile bool facingFarBeacon  = false;   // true when aligned with far beacon
volatile bool facingNearBeacon = false;   // true when aligned with near beacon

volatile bool finishedRotating = false;

void setup() {
  Serial.begin(9600);
  pinMode(sensorPin, INPUT);
}

void loop() {
    if (!finishedRotating) {
        int raw = analogRead(sensorPin); //0-1023 

        float voltage = (raw * 5.0) / 1023.0; //convert to volts 

        bool isFarBeacon  = (voltage >= FAR_MIN_V  && voltage <= FAR_MAX_V);
        bool isNearBeacon = (voltage >= NEAR_MIN_V && voltage <= NEAR_MAX_V);

        facingFarBeacon  = isFarBeacon && !isNearBeacon;  
        facingNearBeacon = isNearBeacon;

        Serial.print("V = ");
        Serial.print(voltage, 3);
        Serial.print("far=");
        Serial.print(facingFarBeacon);
        Serial.print("near=");
        Serial.println(facingNearBeacon);
    } 
    else{ 
        facingFarBeacon  = false;
        facingNearBeacon = false;
    }
    delay(5);
}

