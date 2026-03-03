#include <MecanumWheels.h>

// MecanumWheels(FL_IN1, FL_IN2, FL_ENA,
//               FR_IN1, FR_IN2, FR_ENA,
//               BL_IN1, BL_IN2, BL_ENA,
//               BR_IN1, BR_IN2, BR_ENA)

MecanumWheels robot(
    7,  6,  5,   // Front Left:  IN1, IN2, ENA
    12, 13, 11,  // Front Right: IN1, IN2, ENA
    4,  2,  3,   // Back Left:   IN1, IN2, ENA
    10, 8,  9    // Back Right:  IN1, IN2, ENA
);

void setup() {
    Serial.begin(9600);
    // begin(FL_duty, FR_duty, BL_duty, BR_duty)
    robot.begin(125, 91, 95, 105);
}

void loop() {
    robot.forward();   delay(1000);
    robot.stop();      delay(500);
    robot.backward();  delay(1000);
    robot.stop();      delay(500);
    robot.shiftLeft(); delay(1000);
    robot.stop();      delay(500);
    robot.shiftRight();delay(1000);
    robot.stop();      delay(500);
    robot.turnLeft();  delay(1000);
    robot.stop();      delay(500);
    robot.turnRight(); delay(1000);
    robot.stop();      delay(2000);
}
