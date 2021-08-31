#include "vnh7100motor.h"

VNH7100Motor::VNH7100Motor(unsigned char INA,
                           unsigned char INB,
                           unsigned char PWM,
                           unsigned char SEL0,
                           unsigned char CS)
{
  _INA = INA;
  _INB = INB;
  _PWM = PWM;
  _SEL0 = SEL0;
  _CS = CS;
}

byte VNH7100Motor::init()
{
  int cs = 0;
  byte fault = 0x00;

  digitalWrite(_INA, LOW);
  digitalWrite(_INB, LOW);
  digitalWrite(_PWM, LOW);
  digitalWrite(_SEL0, LOW);

  pinMode(_INA, OUTPUT);
  pinMode(_INB, OUTPUT);
  pinMode(_SEL0, OUTPUT);

  digitalWrite(_CS, LOW);
  pinMode(_CS, INPUT);

  pinMode(_PWM, OUTPUT);

  cli();
  TCB0_CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  TCB1_CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  TCB2_CTRLA = (TCB_CLKSEL_CLKDIV2_gc) | (TCB_ENABLE_bm);
  sei();

  delay(500);

  cs = analogRead(_CS);
  if (cs > 10)
  {
    //error on voutA
    fault = 0X01;
  }

  digitalWrite(_SEL0, HIGH);
  delay(50);

  cs = analogRead(_CS);
  if (cs > 10)
  {
    //error on voutB
    fault = 0x04|fault;
  }

  if(fault==0)
  {
    MState = MSTOP;
  }
  else
  {
    MState = MSTOP;
  }

  return fault;
}

void VNH7100Motor::forward()
{
  if (MState==MSTOP)
  {
    MState = MFORWARD;

    digitalWrite(_INA, HIGH);
    digitalWrite(_INB, LOW);
    digitalWrite(_SEL0, HIGH);
    pwm_value = 5;
    setMSpeed();
  }
}

void VNH7100Motor::backward()
{
  if (MState==MSTOP)
  {
    MState = MBACKWARD;

    digitalWrite(_INA, LOW);
    digitalWrite(_INB, HIGH);
    digitalWrite(_SEL0, LOW);
    pwm_value = 5;
    setMSpeed();
  }
}

void VNH7100Motor::stop()
{
  MState = MSTOP;

  pwm_value = 0;
  setMSpeed();
  digitalWrite(_INA, LOW);
  digitalWrite(_INB, LOW);
  digitalWrite(_SEL0,LOW);
}

void VNH7100Motor::brake()
{
  MState = MBRAKE;

  pwm_value = 100;
  setMSpeed();
  digitalWrite(_INA, HIGH);
  digitalWrite(_INB, HIGH);
  digitalWrite(_SEL0,LOW);
}

void VNH7100Motor::setMSpeed()
{
  analogWrite(_PWM, pwm_value);
}

void VNH7100Motor::setMSpeed(int value)
{
  analogWrite(_PWM, value);
}

int VNH7100Motor::getCurrent()
{
  return analogRead(_CS);
}