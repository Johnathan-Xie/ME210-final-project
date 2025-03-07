#include <Arduino.h>
#include "Motor.h"
#include "ArduinoLog.h"

#define PWM_MAX_VALUE 255
#define PWM_MIN_VALUE 130

Motor::Motor(uint8_t direction_pin_0, uint8_t direction_pin_1, uint8_t pwm_pin, bool reversed = false){
  this->direction_pin_0 = direction_pin_0;
  this->direction_pin_1 = direction_pin_1;
  this->pwm_pin = pwm_pin;
  this->reversed = reversed;

  pinMode(direction_pin_0, OUTPUT);
  pinMode(direction_pin_1, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
}
// Speed must range between -1 (full backward) and 1 (full forward)
void Motor::set_speed(float speed) {
  bool forward;
  int pwm_value;
  //Log.infoln("speed: %F", speed);
  if (speed == 0) {
    digitalWrite(this->direction_pin_0, HIGH);
    digitalWrite(this->direction_pin_1, HIGH);
    analogWrite(this->pwm_pin, 0);
  } else {
      if (speed > 0) {
        forward = true;
      } else {
        forward = false;
      }
      pwm_value = (int)((PWM_MAX_VALUE - PWM_MIN_VALUE) * abs(speed) + PWM_MIN_VALUE);
      if (this->reversed) {
        forward = !forward;
      }
      //Log.infoln("abs(pwm_value - this->previous_pwm_value) >= this->min_pwm_value_change %d", abs(pwm_value - this->previous_pwm_value) >= this->min_pwm_value_change);
      //Log.infoln("this->previous_pwm_value < 0 %d", abs(pwm_value - this->previous_pwm_value) >= this->min_pwm_value_change);
      //Log.infoln("forward != this->previous_forward %d", abs(pwm_value - this->previous_pwm_value) >= this->min_pwm_value_change);
      if (abs(pwm_value - this->previous_pwm_value) >= this->min_pwm_value_change || this->previous_pwm_value < 0 || forward != this->previous_forward) {
        analogWrite(this->pwm_pin, pwm_value);
        //Log.infoln("%d", pwm_value);
        if (forward) {
          digitalWrite(this->direction_pin_0, LOW);
          digitalWrite(this->direction_pin_1, HIGH);
        } else {
          digitalWrite(this->direction_pin_0, HIGH);
          digitalWrite(this->direction_pin_1, LOW);
        }
        this->previous_pwm_value = pwm_value;
        this->previous_forward = forward;
      }
  }
}