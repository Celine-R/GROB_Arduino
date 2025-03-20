#include "BTS7960.h"

BTS7960::BTS7960(uint8_t L_EN, uint8_t R_EN, uint8_t L_PWM, uint8_t R_PWM){
  _R_PWM = R_PWM;
  _L_PWM = L_PWM;
  _L_EN = L_EN;
  _R_EN = R_EN;
  pinMode(_R_PWM, OUTPUT);
  pinMode(_L_PWM, OUTPUT);
  pinMode(_L_EN, OUTPUT);
  pinMode(_R_EN, OUTPUT);
}

void BTS7960::Crush(uint8_t pwm){
     Enable();
     analogWrite(_L_PWM, 0);
     delayMicroseconds(100);
     analogWrite(_R_PWM, pwm);
}

void BTS7960::Uncrush(uint8_t pwm){
     Enable();
     analogWrite(_R_PWM, 0);
     delayMicroseconds(100);
     analogWrite(_L_PWM, pwm);
}

void BTS7960::Enable(){
  digitalWrite(_L_EN,HIGH);
  digitalWrite(_R_EN, HIGH);  
}

void BTS7960::Disable(){
  digitalWrite(_L_EN,LOW);
  digitalWrite(_R_EN, LOW);
}

void BTS7960::Stop(){
  analogWrite(_L_PWM, LOW);
  analogWrite(_R_PWM, LOW);
  Disable();
}
