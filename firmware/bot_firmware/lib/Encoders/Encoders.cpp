#include "Encoders.h"

namespace
{
// Tick counters are updated inside ISRs, so they must be volatile.
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;
}

Encoders::Encoders(
  int left_pin,
  int right_pin,
  int pulses_per_revolution,
  unsigned long print_interval_ms
) :
  left_pin_(left_pin),
  right_pin_(right_pin),
  pulses_per_revolution_(pulses_per_revolution),
  print_interval_ms_(print_interval_ms),
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

void Encoders::printData()
{
  unsigned long current_time_ms = millis();

  // Keep Serial output readable and avoid slowing the main control loop.
  if (current_time_ms - previous_time_ms_ < print_interval_ms_)
  {
    return;
  }

  EncoderReading reading = read();

  Serial.print("Left ticks: ");
  Serial.print(reading.left_ticks);
  Serial.print(" | Right ticks: ");
  Serial.print(reading.right_ticks);
  Serial.print(" | Left RPM: ");
  Serial.print(reading.left_rpm);
  Serial.print(" | Right RPM: ");
  Serial.println(reading.right_rpm);
}

void IRAM_ATTR Encoders::leftISR()
{
  // Keep ISRs tiny: only count the pulse and return.
  left_encoder_ticks++;
}

void IRAM_ATTR Encoders::rightISR()
{
  // Keep ISRs tiny: only count the pulse and return.
  right_encoder_ticks++;
}
