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
    bool reversed;
};