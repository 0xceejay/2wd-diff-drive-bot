#include "IMU.h"

MyIMU::MyIMU(uint8_t i2c_address, TwoWire * wire) :
  i2c_address_(i2c_address),
  wire_(wire),
  initialized_(false),
  gyro_bias_x_(0.0F),
  gyro_bias_y_(0.0F),
  gyro_bias_z_(0.0F)
{
}

bool MyIMU::setupIMU()
{
  if (wire_ == nullptr)
  {
    return false;
  }

  initialized_ = mpu_.begin(i2c_address_, wire_);

  if (!initialized_)
  {
    return false;
  }

  // These ranges provide useful headroom for a small differential-drive robot.
  mpu_.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu_.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu_.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // A new initialization invalidates any bias measured previously.
  gyro_bias_x_ = 0.0F;
  gyro_bias_y_ = 0.0F;
  gyro_bias_z_ = 0.0F;

  return true;
}

bool MyIMU::calibrateGyro(
  uint16_t samples,
  uint16_t sample_delay_ms
)
{
  if (!initialized_ || samples == 0)
  {
    return false;
  }

  float sum_x = 0.0F;
  float sum_y = 0.0F;
  float sum_z = 0.0F;

  for (uint16_t sample = 0; sample < samples; ++sample)
  {
    sensors_event_t acceleration;
    sensors_event_t gyro;
    sensors_event_t temperature;

    if (!mpu_.getEvent(&acceleration, &gyro, &temperature))
    {
      return false;
    }

    sum_x += gyro.gyro.x;
    sum_y += gyro.gyro.y;
    sum_z += gyro.gyro.z;

    if (sample_delay_ms > 0)
    {
      delay(sample_delay_ms);
    }
  }

  gyro_bias_x_ = sum_x / samples;
  gyro_bias_y_ = sum_y / samples;
  gyro_bias_z_ = sum_z / samples;

  return true;
}

bool MyIMU::readIMU(IMUReading & reading)
{
  if (!initialized_)
  {
    return false;
  }

  sensors_event_t acceleration;
  sensors_event_t gyro;
  sensors_event_t temperature;

  if (!mpu_.getEvent(&acceleration, &gyro, &temperature))
  {
    return false;
  }

  reading.acceleration_x = acceleration.acceleration.x;
  reading.acceleration_y = acceleration.acceleration.y;
  reading.acceleration_z = acceleration.acceleration.z;

  reading.angular_velocity_x = gyro.gyro.x - gyro_bias_x_;
  reading.angular_velocity_y = gyro.gyro.y - gyro_bias_y_;
  reading.angular_velocity_z = gyro.gyro.z - gyro_bias_z_;

  reading.temperature = temperature.temperature;

  return true;
}

bool MyIMU::isInitialized() const
{
  return initialized_;
}
