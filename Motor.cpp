#include <Arduino.h>
#include "Motor.h"
#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 52

Motor::Motor(uint8_t pin0, uint8_t pin1){
  this->pin0 = pin0;
  this->pin1 = pin1;
  pinMode(pin0, OUTPUT);
  pinMode(pin1, OUTPUT);
}
// Speed must range between -1 (full backward) and 1 (full forward)
void Motor::setSpeed(float speed) {
  bool forward;
  int pwmValue;
  if (speed > 0) {
    forward = true;
    pwmValue = (int)((PWM_MAX_VALUE - PWM_MIN_VALUE) * speed + PWM_MIN_VALUE);
  } else {
    forward = false;
    pwmValue = (int)((PWM_MAX_VALUE - PWM_MIN_VALUE) * speed + PWM_MIN_VALUE);
  }
  if (pwmValue == PWM_MIN_VALUE) {
    analogWrite(this->pin0, PWM_MAX_VALUE);
    analogWrite(this->pin1, PWM_MAX_VALUE);
  } else {
    if (forward) {
      analogWrite(this->pin0, 0);
      analogWrite(this->pin1, PWM_MAX_VALUE);
      delay(10);
      analogWrite(this->pin1, pwmValue);
    } else {
      analogWrite(this->pin0, PWM_MAX_VALUE);
      delay(10);
      analogWrite(this->pin0, pwmValue);
      analogWrite(this->pin1, 0);
    }
  }
  Serial.println(pwmValue);
}