#include "Encoders.h"

namespace
{
// Tick counters are updated inside ISRs, so they must be volatile.
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;
volatile int left_encoder_direction = 0;
volatile int right_encoder_direction = 0;
}

Encoders::Encoders(
  int left_pin,
  int right_pin,
  int pulses_per_revolution
) :
  left_pin_(left_pin),
  right_pin_(right_pin),
  pulses_per_revolution_(pulses_per_revolution),
  previous_left_ticks_(0),
  previous_right_ticks_(0),
  previous_time_ms_(0)
{
}

void Encoders::begin()
{
  // HC-020K outputs are read as simple digital pulse inputs.
  pinMode(left_pin_, INPUT);
  pinMode(right_pin_, INPUT);

  // Count one tick whenever the sensor output falls.
  attachInterrupt(
    digitalPinToInterrupt(left_pin_),
    leftISR,
    FALLING
  );

  attachInterrupt(
    digitalPinToInterrupt(right_pin_),
    rightISR,
    FALLING
  );

  previous_time_ms_ = millis();
}

void Encoders::setDirections(int left_direction, int right_direction)
{
  // Normalize arbitrary inputs so the interrupt handlers only use -1, 0, or 1.
  int normalized_left =
    left_direction > 0 ? 1 : (left_direction < 0 ? -1 : 0);
  int normalized_right =
    right_direction > 0 ? 1 : (right_direction < 0 ? -1 : 0);

  // Direction is shared with the interrupt handlers, so update it atomically.
  noInterrupts();
  left_encoder_direction = normalized_left;
  right_encoder_direction = normalized_right;
  interrupts();
}

EncoderReading Encoders::read()
{
  long current_left_ticks;
  long current_right_ticks;

  // Copy shared ISR counters atomically so each reading is consistent.
  noInterrupts();
  current_left_ticks = left_encoder_ticks;
  current_right_ticks = right_encoder_ticks;
  interrupts();

  unsigned long current_time_ms = millis();
  unsigned long elapsed_ms = current_time_ms - previous_time_ms_;
  float dt = elapsed_ms / 1000.0;

  long left_delta = current_left_ticks - previous_left_ticks_;
  long right_delta = current_right_ticks - previous_right_ticks_;

  // Always return total ticks, even if not enough time has passed for RPM.
  EncoderReading reading;
  reading.left_ticks = current_left_ticks;
  reading.right_ticks = current_right_ticks;
  reading.left_rpm = 0.0;
  reading.right_rpm = 0.0;

  if (dt > 0.0)
  {
    // RPM = revolutions per sample * samples per minute.
    reading.left_rpm =
      (left_delta / (float)pulses_per_revolution_) * (60.0 / dt);
    reading.right_rpm =
      (right_delta / (float)pulses_per_revolution_) * (60.0 / dt);
  }

  previous_left_ticks_ = current_left_ticks;
  previous_right_ticks_ = current_right_ticks;
  previous_time_ms_ = current_time_ms;

  return reading;
}

void IRAM_ATTR Encoders::leftISR()
{
  // Infer signed motion from the direction most recently commanded to the motor.
  left_encoder_ticks += left_encoder_direction;
}

void IRAM_ATTR Encoders::rightISR()
{
  right_encoder_ticks += right_encoder_direction;
}
