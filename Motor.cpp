#include <Arduino.h>
#include "Motor.h"
#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 52

Motor::Motor(uint8_t pin0, uint8_t pin1, bool reversed = false){
  this->pin0 = pin0;
  this->pin1 = pin1;
  this->reversed = reversed;
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
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
    analogWrite(this->pin0, PWM_MAX_VALUE);
    analogWrite(this->pin1, PWM_MAX_VALUE);
  } else {
    if (forward) {
      analogWrite(this->pin0, 0);
      analogWrite(this->pin1, pwm_value);
    } else {
      analogWrite(this->pin0, pwm_value);
      analogWrite(this->pin1, 0);
    }
  }
  Serial.println(pwm_value);
}