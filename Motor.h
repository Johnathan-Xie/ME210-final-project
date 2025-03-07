#pragma once
// Potentially add provision for integrating for location
class Motor {
  public:
    Motor::Motor(uint8_t direction_pin_0, uint8_t direction_pin_1, uint8_t pwm_pin, bool reversed = false);
    void set_speed(float speed);

  private:
    uint8_t direction_pin_0;
    uint8_t direction_pin_1;
    uint8_t pwm_pin;
    int previous_pwm_value = -1;
    bool previous_forward = true;
    int min_pwm_value_change = 5;
    bool reversed;
};