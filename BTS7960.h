#ifndef BTS7960_h
#define BTS7960_h
#include "Arduino.h"

class BTS7960
{
  public:
    BTS7960(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM);
    void Enable();
    void Disable();

    void Crush(uint8_t pwm);
    void Uncrush(uint8_t pwm);
    void Stop();

  private:
    uint8_t _L_EN;
    uint8_t _R_EN;
    uint8_t _L_PWM;
    uint8_t _R_PWM;
};
#endif
