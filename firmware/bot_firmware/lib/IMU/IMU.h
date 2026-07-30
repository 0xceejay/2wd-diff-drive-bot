#ifndef IMU_H
#define IMU_H

#include <Adafruit_MPU6050.h>
#include <Arduino.h>
#include <Wire.h>

// Acceleration is in m/s^2, angular velocity is in rad/s, and temperature is
// in degrees Celsius. These are the SI units returned by Adafruit's library.
struct IMUReading
{
  float acceleration_x;
  float acceleration_y;
  float acceleration_z;
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  float temperature;
};

class MyIMU
{
public:
  // Most MPU6050 boards use address 0x68. If AD0 is HIGH, pass 0x69 instead.
  explicit MyIMU(
    uint8_t i2c_address = MPU6050_I2CADDR_DEFAULT,
    TwoWire * wire = &Wire
  );

  // Initialize the sensor and configure ranges suitable for a mobile robot.
  // Returns false when the MPU6050 cannot be found on the I2C bus.
  bool setupIMU();

  // Measure and save the stationary gyro bias. Keep the robot completely still
  // while this runs. Returns false if setup has not succeeded or a read fails.
  bool calibrateGyro(
    uint16_t samples = 500,
    uint16_t sample_delay_ms = 2
  );

  // Read the latest sensor sample and apply the saved gyro bias.
  bool readIMU(IMUReading & reading);

  bool isInitialized() const;

private:
  Adafruit_MPU6050 mpu_;
  uint8_t i2c_address_;
  TwoWire * wire_;
  bool initialized_;
  float gyro_bias_x_;
  float gyro_bias_y_;
  float gyro_bias_z_;
};

#endif
