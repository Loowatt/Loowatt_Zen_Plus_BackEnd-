#pragma once

#include <Arduino.h>

enum m_StateType{MSTOP, MBACKWARD, MFORWARD, MBRAKE};

class VNH7100Motor
{
  public:
    // CONSTRUCTOR
    // User-defined pin selection.
    VNH7100Motor(unsigned char INA,
                 unsigned char INB,
                 unsigned char PWM,
                 unsigned char SEL0,
                 unsigned char CS);

    // PUBLIC METHODS
    byte init();
    void forward();
    void backward();
    void stop();
    void brake();
    void setMSpeed();
    void setMSpeed(int value);
    int getCurrent();

    int pwm_value = 0;
    m_StateType MState;
    double RPM = 0.0;

  private:
    unsigned char _INA;
    unsigned char _INB;
    unsigned char _PWM;
    unsigned char _SEL0;
    unsigned char _CS;
};