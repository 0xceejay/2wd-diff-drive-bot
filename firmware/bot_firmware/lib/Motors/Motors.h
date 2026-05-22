#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

// Motor A is the left wheel and motor B the right wheel.
struct MotorPins
{
  int pwm_a;
  int ain1;
  int ain2;
  int pwm_b;
  int bin1;
  int bin2;
  int standby;
};

class Motors
{
public:
  // pwm_frequency/resolution/channels configure ESP32 LEDC PWM output.
  Motors(
    const MotorPins & pins,
    int pwm_frequency,
    int pwm_resolution,
    int pwm_channel_a,
    int pwm_channel_b
  );

  // Configure motor direction pins, standby pin, and PWM channels.
  void begin();

  // pwm range: -255 reverse, 0 brake/stop, 255 forward.
  void setLeft(int pwm);
  void setRight(int pwm);

  // Brake both motors by setting both input pins HIGH with zero PWM.
  void stop();

private:
  MotorPins pins_;
  int pwm_frequency_;
  int pwm_resolution_;
  int pwm_channel_a_;
  int pwm_channel_b_;
};

#endif
