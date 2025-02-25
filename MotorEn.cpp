#include <Arduino.h>
#include "Motor.h"
#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 52

Motor::Motor(uint8_t direction_pin0, uint8_t direction_pin1, uint8_t pwm_pin, bool reversed = false){
  this->direction_pin0 = direction_pin0;
  this->direction_pin1 = direction_pin1;
  this->pwm_pin = pwm_pin;
  this->reversed = reversed;

  pinMode(direction_pin0, OUTPUT);
  pinMode(direction_pin1, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}
// Speed must range between -1 (full backward) and 1 (full forward)
void Motor::set_speed(float speed) {
  bool forward;
  int pwm_value;
  if (speed > 0) {
    forward = true;
    pwm_value = (int)((PWM_MAX_VALUE - PWM_MIN_VALUE) * speed + PWM_MIN_VALUE);
  } else {
    forward = false;
    pwm_value = (int)((PWM_MAX_VALUE - PWM_MIN_VALUE) * speed + PWM_MIN_VALUE);
  }
  if (this->reversed) {
    forward = !forward;
  }
  
  if (pwm_value == PWM_MIN_VALUE) {
    digitalWrite(this->direction_pin0, HIGH);
    digitalWrite(this->direction_pin1, HIGH);
    analogWrite(this->pwm_pin, 0);
  } else {
    analogWrite(this->pwm_pin, pwm_value);
    if (forward) {
      digitalWrite(this->direction_pin0, LOW);
      digitalWrite(this->direction_pin1, HIGH);
    } else {
      digitalWrite(this->direction_pin0, HIGH);
      digitalWrite(this->direction_pin1, LOW);
    }
  }
  Serial.println(pwm_value);
}