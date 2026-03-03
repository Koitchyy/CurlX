#include "MecanumWheels.h"

MecanumWheels::MecanumWheels(
    int fl_in1, int fl_in2, int fl_ena,
    int fr_in1, int fr_in2, int fr_ena,
    int bl_in1, int bl_in2, int bl_ena,
    int br_in1, int br_in2, int br_ena
) {
    _fl_in1 = fl_in1; _fl_in2 = fl_in2; _fl_ena = fl_ena;
    _fr_in1 = fr_in1; _fr_in2 = fr_in2; _fr_ena = fr_ena;
    _bl_in1 = bl_in1; _bl_in2 = bl_in2; _bl_ena = bl_ena;
    _br_in1 = br_in1; _br_in2 = br_in2; _br_ena = br_ena;
}

void MecanumWheels::begin(int fl_duty, int fr_duty, int bl_duty, int br_duty) {
    _fl_duty = fl_duty;
    _fr_duty = fr_duty;
    _bl_duty = bl_duty;
    _br_duty = br_duty;

    // Front Left
    pinMode(_fl_in1, OUTPUT); pinMode(_fl_in2, OUTPUT); pinMode(_fl_ena, OUTPUT);
    digitalWrite(_fl_in1, LOW); digitalWrite(_fl_in2, LOW);

    // Front Right
    pinMode(_fr_in1, OUTPUT); pinMode(_fr_in2, OUTPUT); pinMode(_fr_ena, OUTPUT);
    digitalWrite(_fr_in1, LOW); digitalWrite(_fr_in2, LOW);

    // Back Left
    pinMode(_bl_in1, OUTPUT); pinMode(_bl_in2, OUTPUT); pinMode(_bl_ena, OUTPUT);
    digitalWrite(_bl_in1, LOW); digitalWrite(_bl_in2, LOW);

    // Back Right
    pinMode(_br_in1, OUTPUT); pinMode(_br_in2, OUTPUT); pinMode(_br_ena, OUTPUT);
    digitalWrite(_br_in1, LOW); digitalWrite(_br_in2, LOW);

    analogWrite(_fl_ena, _fl_duty);
    analogWrite(_fr_ena, _fr_duty);
    analogWrite(_bl_ena, _bl_duty);
    analogWrite(_br_ena, _br_duty);
}

void MecanumWheels::setDuty(int fl_duty, int fr_duty, int bl_duty, int br_duty) {
    _fl_duty = fl_duty; _fr_duty = fr_duty;
    _bl_duty = bl_duty; _br_duty = br_duty;
    analogWrite(_fl_ena, _fl_duty);
    analogWrite(_fr_ena, _fr_duty);
    analogWrite(_bl_ena, _bl_duty);
    analogWrite(_br_ena, _br_duty);
}

void MecanumWheels::forward() {
    CW(_fl_in1,  _fl_in2);
    CCW(_fr_in1, _fr_in2);
    CCW(_bl_in1, _bl_in2);
    CW(_br_in1,  _br_in2);
}

void MecanumWheels::backward() {
    CCW(_fl_in1, _fl_in2);
    CW(_fr_in1,  _fr_in2);
    CW(_bl_in1,  _bl_in2);
    CCW(_br_in1, _br_in2);
}

void MecanumWheels::shiftRight() {
    CW(_fl_in1, _fl_in2);
    CW(_fr_in1, _fr_in2);
    CW(_bl_in1, _bl_in2);
    CW(_br_in1, _br_in2);
}

void MecanumWheels::shiftLeft() {
    CCW(_fl_in1, _fl_in2);
    CCW(_fr_in1, _fr_in2);
    CCW(_bl_in1, _bl_in2);
    CCW(_br_in1, _br_in2);
}

void MecanumWheels::turnRight() {
    CW(_fl_in1,  _fl_in2);
    CW(_fr_in1,  _fr_in2);
    CCW(_bl_in1, _bl_in2);
    CCW(_br_in1, _br_in2);
}

void MecanumWheels::turnLeft() {
    CW(_fl_in1,  _fl_in2);
    CW(_fr_in1,  _fr_in2);
    CCW(_bl_in1, _bl_in2);
    CCW(_br_in1, _br_in2);
}

void MecanumWheels::stop() {
    STOP(_fl_in1, _fl_in2);
    STOP(_fr_in1, _fr_in2);
    STOP(_bl_in1, _bl_in2);
    STOP(_br_in1, _br_in2);
}

// --- Private helpers ---

void MecanumWheels::CW(int in1, int in2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
}

void MecanumWheels::CCW(int in1, int in2) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
}

void MecanumWheels::STOP(int in1, int in2) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
}
