#pragma once
// Potentially add provision for integrating for location
class Motor {
  public:
    Motor::Motor(uint8_t pin0, uint8_t pin1, bool reversed = false);
    void set_speed(float speed);

  private:
    uint8_t pin0;
    uint8_t pin1;
    bool reversed;
};