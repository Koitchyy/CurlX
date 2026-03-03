#ifndef MECANUMWHEELS_H
#define MECANUMWHEELS_H

#include <Arduino.h>

class MecanumWheels {
public:
    // Constructor: pin order is IN1, IN2, ENA for each wheel
    MecanumWheels(
        int fl_in1, int fl_in2, int fl_ena,
        int fr_in1, int fr_in2, int fr_ena,
        int bl_in1, int bl_in2, int bl_ena,
        int br_in1, int br_in2, int br_ena
    );

    // Initialize pins and set duty cycles
    void begin(int fl_duty = 125, int fr_duty = 91, int bl_duty = 95, int br_duty = 105);

    // Movement commands
    void forward();
    void backward();
    void shiftLeft();
    void shiftRight();
    void turnLeft();
    void turnRight();
    void stop();

    // Update individual wheel duty cycles at runtime
    void setDuty(int fl_duty, int fr_duty, int bl_duty, int br_duty);

private:
    int _fl_in1, _fl_in2, _fl_ena;
    int _fr_in1, _fr_in2, _fr_ena;
    int _bl_in1, _bl_in2, _bl_ena;
    int _br_in1, _br_in2, _br_ena;

    int _fl_duty, _fr_duty, _bl_duty, _br_duty;

    void CW(int in1, int in2);
    void CCW(int in1, int in2);
    void STOP(int in1, int in2);
};

#endif
