/*
#include <Servo.h>
#include <Arduino.h>


Servo flag_servo;
Servo igniter_servo;
Servo dump_servo;

const int FLAG_SERVO_ACTIVE_DEGREES = 90;
const int FLAG_SERVO_INACTIVE_DEGREES = 0;

const int IGNITER_SERVO_ACTIVE_DEGREES = 90;
const int IGNITER_SERVO_INACTIVE_DEGREES = 180;

const int DUMP_SERVO_ACTIVE_DEGREES = 60;
const int DUMP_SERVO_INACTIVE_DEGREES = 180;
const int FLAG_PIN = 11;
const int IGNITER_PIN = 5;
const int DUMP_SERVO_PIN = 3;
const unsigned long ACTIVE_MILLISECONDS = 130000; //130000
const unsigned long DUMP_SERVO_MILLISECONDS = 15000;
bool already_activated = false;

void setup() {
    flag_servo.attach(FLAG_PIN);
    igniter_servo.attach(IGNITER_PIN);
    dump_servo.attach(DUMP_SERVO_PIN);

    flag_servo.write(FLAG_SERVO_INACTIVE_DEGREES);
    igniter_servo.write(IGNITER_SERVO_INACTIVE_DEGREES);
    dump_servo.write(DUMP_SERVO_INACTIVE_DEGREES);
    delay(10000);
}

void loop() {
    flag_servo.write(FLAG_SERVO_ACTIVE_DEGREES);
    igniter_servo.write(IGNITER_SERVO_ACTIVE_DEGREES);
    delay(DUMP_SERVO_MILLISECONDS);
    dump_servo.write(DUMP_SERVO_ACTIVE_DEGREES);
    delay(ACTIVE_MILLISECONDS - DUMP_SERVO_MILLISECONDS);
    flag_servo.write(FLAG_SERVO_INACTIVE_DEGREES);
    igniter_servo.write(IGNITER_SERVO_INACTIVE_DEGREES);
}
*/