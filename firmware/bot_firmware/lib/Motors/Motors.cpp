#include "Motors.h"

Motors::Motors(
  const MotorPins & pins,
  int pwm_frequency,
  int pwm_resolution,
  int pwm_channel_a,
  int pwm_channel_b
) :
  pins_(pins),
  pwm_frequency_(pwm_frequency),
  pwm_resolution_(pwm_resolution),
  pwm_channel_a_(pwm_channel_a),
  pwm_channel_b_(pwm_channel_b)
{
}

void Motors::begin()
{
  // Direction and standby pins are normal digital outputs.
  pinMode(pins_.ain1, OUTPUT);
  pinMode(pins_.ain2, OUTPUT);
  pinMode(pins_.bin1, OUTPUT);
  pinMode(pins_.bin2, OUTPUT);
  pinMode(pins_.standby, OUTPUT);

  // STBY must be HIGH or the TB6612FNG keeps both motor outputs disabled.
  digitalWrite(pins_.standby, HIGH);

  // The ESP32 uses LEDC channels to generate PWM on arbitrary GPIO pins.
  ledcSetup(pwm_channel_a_, pwm_frequency_, pwm_resolution_);
  ledcSetup(pwm_channel_b_, pwm_frequency_, pwm_resolution_);

  ledcAttachPin(pins_.pwm_a, pwm_channel_a_);
  ledcAttachPin(pins_.pwm_b, pwm_channel_b_);
}

void Motors::setLeft(int pwm)
{
  // Keep the public API safe even if caller passes a value outside PWM range.
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    digitalWrite(pins_.ain1, HIGH);
    digitalWrite(pins_.ain2, LOW);
  }
  else if (pwm < 0)
  {
    digitalWrite(pins_.ain1, LOW);
    digitalWrite(pins_.ain2, HIGH);
  }
  else
  {
    // High on both pins results in short-brake mode on the TB6612FNG.
    digitalWrite(pins_.ain1, HIGH);
    digitalWrite(pins_.ain2, HIGH);
  }

  ledcWrite(pwm_channel_a_, abs(pwm));
}

void Motors::setRight(int pwm)
{
  // Constrain even if caller passes a value outside PWM range.
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    digitalWrite(pins_.bin1, HIGH);
    digitalWrite(pins_.bin2, LOW);
  }
  else if (pwm < 0)
  {
    digitalWrite(pins_.bin1, LOW);
    digitalWrite(pins_.bin2, HIGH);
  }
  else
  {
    // High on both pins results in short-brake mode on the TB6612FNG.
    digitalWrite(pins_.bin1, HIGH);
    digitalWrite(pins_.bin2, HIGH);
  }

  ledcWrite(pwm_channel_b_, abs(pwm));
}

void Motors::stop()
{
  // Reuse the same stop behavior for both wheels so braking stays consistent.
  setLeft(0);
  setRight(0);
}
