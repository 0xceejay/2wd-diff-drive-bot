#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// Total ticks plus speed since the last reading.
struct EncoderReading
{
  long left_ticks;
  long right_ticks;
  float left_rpm;
  float right_rpm;
};

class Encoders
{
public:
  // pulses_per_revolution is the number of encoder slots/ticks per wheel turn.
  Encoders(
    int left_pin,
    int right_pin,
    int pulses_per_revolution
  );

  // Configure encoder GPIO inputs and attach interrupt handlers.
  void begin();

  // HC-020K sensors provide one pulse channel and cannot determine direction.
  // Supply the direction commanded to each motor: -1, 0, or 1.
  void setDirections(int left_direction, int right_direction);

  // Copy tick counters safely and calculate RPM since the previous read().
  EncoderReading read();

private:
  // Interrupt handlers must be static so attachInterrupt can call them.
  static void IRAM_ATTR leftISR();
  static void IRAM_ATTR rightISR();

  int left_pin_;
  int right_pin_;
  int pulses_per_revolution_;
  long previous_left_ticks_;
  long previous_right_ticks_;
  unsigned long previous_time_ms_;
};

#endif
