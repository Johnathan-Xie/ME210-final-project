/*
#define USE_TIMER_1 true

#include <TimerInterrupt.h>
#include <TimerInterrupt.hpp>
#include <ISR_Timer.h>
#include <ISR_Timer.hpp>
#include <Servo.h>
#include <Arduino.h>

Servo flag_servo;
Servo igniter_servo;
const int FLAG_SERVO_ACTIVE_DEGREES = 90;
const int FLAG_SERVO_INACTIVE_DEGREES 0;

const int IGNITER_SERVO_ACTIVE_DEGREES = 90;
const int IGNITER_SERVO_INACTIVE_DEGREES = 0;

const int FLAG_PIN = A0;
const int IGNITER_PIN = A1;
const int SIGNAL_PIN = 2;
const unsigned long ACTIVE_MILLISECONDS = 130000;

typedef enum {
  STATE_INACTIVE,
  STATE_ACTIVE,
} States_t;
States_t state = STATE_INACTIVE;

//bool interrupt_attached = false;
bool already_activated = false;

void set_inactive(){
    state = STATE_INACTIVE;
}
void setup() {
    flag_servo.attach(FLAG_PIN);
    igniter_servo.attach(IGNITER_PIN);
    pinMode(SIGNAL_PIN);
    ITimer1.init();
}

void loop() {
    switch (state) {
        case STATE_INACTIVE:
            flag_servo.write(FLAG_SERVO_INACTIVE_DEGREES);
            igniter_servo.write(IGNITER_SERVO_INACTIVE_DEGREES);
            if (digitalRead(SIGNAL_PIN) == HIGH && !already_activated) {
                state = STATE_ACTIVE;
                already_activated = true;
                if (!ITimer1.attachInterruptInterval(ACTIVE_MILLISECONDS, set_inactive)) {
                    Serial.println("Could not attach interrupt for %d", ACTIVE_MILLISECONDS);
                }
            }
            break;
        case STATE_ACTIVE:
            flag_servo.write(FLAG_SERVO_ACTIVE_DEGREES);
            igniter_servo.write(IGNITER_SERVO_ACTIVE_DEGREES);
            break;
    }
}
*/