#include <Arduino.h>
#include <Encoders.h>
#include <IMU.h>
#include <Motors.h>
#include <micro_ros_platformio.h>

// Core ROS 2 C libraries
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/temperature.h>

// WIFI credentials
#include "wifi_secrets.hpp"

// IP address
IPAddress agent_ip(
  AGENT_IP_1,
  AGENT_IP_2,
  AGENT_IP_3,
  AGENT_IP_4
);

// UDP port used by micro-ROS agent
size_t agent_port = 8888;

// ROS OBJECTS
rcl_allocator_t allocator;
rclc_executor_t executor;
rclc_support_t support;
rcl_subscription_t subscriber;
rcl_publisher_t imu_publisher;
rcl_publisher_t temperature_publisher;
rcl_publisher_t joint_state_publisher;
rcl_node_t node;
geometry_msgs__msg__Twist cmd_vel_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__Temperature temperature_msg;
sensor_msgs__msg__JointState joint_state_msg;

// Robot geometry
const float WHEEL_RADIUS = 0.033;
const float WHEEL_BASE = 0.1443;

// Wheel-speed targets calculated from the most recent /cmd_vel command.
float left_wheel_target_velocity = 0.0F;
float right_wheel_target_velocity = 0.0F;

// PWM settings
const int PWM_FREQ = 20000;

// PWM resolution in bits
// 8 bits (values from 0 to 255)
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// Speed limit
const int MAX_PWM = 110;
const int MIN_MOVING_PWM = 45;
const unsigned long CMD_VEL_TIMEOUT_MS = 300;
const unsigned long IMU_READ_INTERVAL_MS = 100;
const unsigned long WHEEL_CONTROL_INTERVAL_MS = 100;
const unsigned long ENCODER_PUBLISH_INTERVAL_MS = 100;
const float TWO_PI_F = 6.28318530718F;
const float ENCODER_PULSES_PER_REVOLUTION = 20.0F;
const float MAX_WHEEL_SPEED_MPS = 0.5F;
const float STOPPED_VELOCITY_THRESHOLD_MPS = 0.005F;

// Initial velocity-controller gains.
const float WHEEL_PID_KP = 80.0F;
const float WHEEL_PID_KI = 30.0F;
const float WHEEL_PID_KD = 2.0F;
const float WHEEL_PID_INTEGRAL_LIMIT = 1.0F;

struct WheelPIDState
{
  float integral;
  float previous_error;
  bool has_previous_error;
};

MotorPins motor_pins = {
  25, // PWMA
  26, // AIN1
  27, // AIN2
  13, // PWMB
  33, // BIN1
  32, // BIN2
  14  // STBY
};

Motors motors(
  motor_pins,
  PWM_FREQ,
  PWM_RESOLUTION,
  PWM_CHANNEL_A,
  PWM_CHANNEL_B
);

Encoders encoders(
  34,  // Left HC-020K encoder pin
  35,  // Right HC-020K encoder pin
  (int)ENCODER_PULSES_PER_REVOLUTION
);

// The ESP32's default I2C pins are GPIO 21 (SDA) and GPIO 22 (SCL).
MyIMU imu;
IMUReading imu_reading;

// Timestamp of last received /cmd_vel message
unsigned long last_cmd_vel_ms = 0;
unsigned long last_imu_read_ms = 0;
unsigned long last_wheel_control_ms = 0;
unsigned long last_encoder_publish_ms = 0;

// Flag to track if we've received at least one /cmd_vel message
bool has_received_cmd_vel = false;
bool imu_available = false;
bool encoder_reading_available = false;

WheelPIDState left_wheel_pid = {0.0F, 0.0F, false};
WheelPIDState right_wheel_pid = {0.0F, 0.0F, false};
EncoderReading latest_encoder_reading = {0, 0, 0.0F, 0.0F};

// Stop the motors and halt execution after an unrecoverable startup failure.
void stopOnFatalError()
{
  motors.stop();

  while (true)
  {
    delay(1000);
  }
}

// Stamp a ROS message with synchronized agent time or local uptime as fallback.
void setMessageTimestamp(std_msgs__msg__Header & header)
{
  int64_t timestamp_ns = rmw_uros_epoch_nanos();

  // Fall back to time since boot if session time synchronization failed.
  if (timestamp_ns <= 0)
  {
    timestamp_ns = (int64_t)millis() * 1000000LL;
  }

  header.stamp.sec = (int32_t)(timestamp_ns / 1000000000LL);
  header.stamp.nanosec = (uint32_t)(timestamp_ns % 1000000000LL);
}

// Raise nonzero PWM commands to the minimum value that can move a wheel.
int applyMinimumMovingPwm(int pwm)
{
  if (pwm == 0)
  {
    return 0;
  }

  if (abs(pwm) < MIN_MOVING_PWM)
  {
    return pwm > 0 ? MIN_MOVING_PWM : -MIN_MOVING_PWM;
  }

  return pwm;
}

// Clear accumulated error and derivative history for one wheel controller.
void resetWheelPID(WheelPIDState & state)
{
  state.integral = 0.0F;
  state.previous_error = 0.0F;
  state.has_previous_error = false;
}

// Convert a wheel-velocity error into a bounded feed-forward plus PID command.
int calculateWheelPwm(
  float target_velocity,
  float measured_velocity,
  float dt,
  WheelPIDState & state
)
{
  // A zero target uses active braking and clears accumulated controller state.
  if (abs(target_velocity) < STOPPED_VELOCITY_THRESHOLD_MPS)
  {
    resetWheelPID(state);
    return 0;
  }

  float error = target_velocity - measured_velocity;

  // Clamp the integral term to prevent wind-up while the output is saturated.
  state.integral += error * dt;
  state.integral = constrain(
    state.integral,
    -WHEEL_PID_INTEGRAL_LIMIT,
    WHEEL_PID_INTEGRAL_LIMIT
  );

  float derivative = 0.0F;

  if (state.has_previous_error && dt > 0.0F)
  {
    derivative = (error - state.previous_error) / dt;
  }

  state.previous_error = error;
  state.has_previous_error = true;

  // Feed-forward supplies the approximate steady-state PWM. PID then corrects
  // for battery voltage, motor mismatch, load, and floor-friction changes.
  float feedforward =
    (target_velocity / MAX_WHEEL_SPEED_MPS) * MAX_PWM;
  float correction =
    WHEEL_PID_KP * error +
    WHEEL_PID_KI * state.integral +
    WHEEL_PID_KD * derivative;
  float requested_pwm = feedforward + correction;

  // Do not command the wheel to reverse merely to correct a brief overspeed.
  if (target_velocity > 0.0F)
  {
    requested_pwm = constrain(requested_pwm, 0.0F, (float)MAX_PWM);
  }
  else
  {
    requested_pwm = constrain(requested_pwm, (float)-MAX_PWM, 0.0F);
  }

  return applyMinimumMovingPwm((int)requested_pwm);
}

// Sample both encoders and update the closed-loop motor commands periodically.
void updateWheelVelocityControl()
{
  unsigned long current_time_ms = millis();
  unsigned long elapsed_ms = current_time_ms - last_wheel_control_ms;

  if (elapsed_ms < WHEEL_CONTROL_INTERVAL_MS)
  {
    return;
  }

  last_wheel_control_ms = current_time_ms;
  float dt = elapsed_ms / 1000.0F;

  // One encoder sample drives both the controller and the ROS publication.
  latest_encoder_reading = encoders.read();
  encoder_reading_available = true;

  float left_measured_velocity =
    latest_encoder_reading.left_rpm * TWO_PI_F * WHEEL_RADIUS / 60.0F;
  float right_measured_velocity =
    latest_encoder_reading.right_rpm * TWO_PI_F * WHEEL_RADIUS / 60.0F;

  int left_pwm = calculateWheelPwm(
    left_wheel_target_velocity,
    left_measured_velocity,
    dt,
    left_wheel_pid
  );
  int right_pwm = calculateWheelPwm(
    right_wheel_target_velocity,
    right_measured_velocity,
    dt,
    right_wheel_pid
  );

  // Single-channel encoders need the commanded direction to sign each pulse.
  encoders.setDirections(left_pwm, right_pwm);
  motors.setLeft(left_pwm);
  motors.setRight(right_pwm);
}

// Read and publish acceleration, angular velocity, and temperature periodically.
void readAndPublishIMU()
{
  if (!imu_available)
  {
    return;
  }

  unsigned long current_time_ms = millis();

  if (current_time_ms - last_imu_read_ms < IMU_READ_INTERVAL_MS)
  {
    return;
  }

  last_imu_read_ms = current_time_ms;

  if (!imu.readIMU(imu_reading))
  {
    return;
  }

  setMessageTimestamp(imu_msg.header);
  temperature_msg.header.stamp = imu_msg.header.stamp;

  imu_msg.linear_acceleration.x = imu_reading.acceleration_x;
  imu_msg.linear_acceleration.y = imu_reading.acceleration_y;
  imu_msg.linear_acceleration.z = imu_reading.acceleration_z;
  imu_msg.angular_velocity.x = imu_reading.angular_velocity_x;
  imu_msg.angular_velocity.y = imu_reading.angular_velocity_y;
  imu_msg.angular_velocity.z = imu_reading.angular_velocity_z;
  temperature_msg.temperature = imu_reading.temperature;

  if (rcl_publish(&imu_publisher, &imu_msg, NULL) != RCL_RET_OK)
  {
    return;
  }

  if (rcl_publish(
        &temperature_publisher,
        &temperature_msg,
        NULL
      ) != RCL_RET_OK)
  {
    return;
  }
}

// Publish the latest encoder-derived wheel positions and angular velocities.
void readAndPublishEncoders()
{
  unsigned long current_time_ms = millis();

  if (!encoder_reading_available ||
      current_time_ms - last_encoder_publish_ms <
        ENCODER_PUBLISH_INTERVAL_MS)
  {
    return;
  }

  last_encoder_publish_ms = current_time_ms;

  setMessageTimestamp(joint_state_msg.header);

  joint_state_msg.position.data[0] =
    (latest_encoder_reading.left_ticks / ENCODER_PULSES_PER_REVOLUTION) *
    TWO_PI_F;
  joint_state_msg.position.data[1] =
    (latest_encoder_reading.right_ticks / ENCODER_PULSES_PER_REVOLUTION) *
    TWO_PI_F;

  joint_state_msg.velocity.data[0] =
    latest_encoder_reading.left_rpm * TWO_PI_F / 60.0F;
  joint_state_msg.velocity.data[1] =
    latest_encoder_reading.right_rpm * TWO_PI_F / 60.0F;

  if (rcl_publish(
        &joint_state_publisher,
        &joint_state_msg,
        NULL
      ) != RCL_RET_OK)
  {
    return;
  }
}

// Allocate message storage and populate fixed frame and joint metadata.
void initializeMessages()
{
  if (!geometry_msgs__msg__Twist__init(&cmd_vel_msg) ||
      !sensor_msgs__msg__Imu__init(&imu_msg) ||
      !sensor_msgs__msg__Temperature__init(&temperature_msg) ||
      !sensor_msgs__msg__JointState__init(&joint_state_msg))
  {
    stopOnFatalError();
  }

  if (!rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "base_link") ||
      !rosidl_runtime_c__String__assign(
        &temperature_msg.header.frame_id,
        "base_link"
      ) ||
      !rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 2) ||
      !rosidl_runtime_c__double__Sequence__init(
        &joint_state_msg.position,
        2
      ) ||
      !rosidl_runtime_c__double__Sequence__init(
        &joint_state_msg.velocity,
        2
      ))
  {
    stopOnFatalError();
  }

  if (!rosidl_runtime_c__String__assign(
        &joint_state_msg.name.data[0],
        "left_wheel"
      ) ||
      !rosidl_runtime_c__String__assign(
        &joint_state_msg.name.data[1],
        "right_wheel"
      ))
  {
    stopOnFatalError();
  }

  // The MPU6050 wrapper does not calculate orientation.
  imu_msg.orientation_covariance[0] = -1.0;
}

// Convert each received velocity command into left and right wheel targets.
void cmd_vel_callback(const void * msgin)
{
  // Cast incoming generic pointer to Twist message type
  const geometry_msgs__msg__Twist * twist_msg =
    (const geometry_msgs__msg__Twist *)msgin;

  float linear_velocity = twist_msg->linear.x;
  float angular_velocity = twist_msg->angular.z;

  left_wheel_target_velocity =
    linear_velocity - (WHEEL_BASE / 2.0F) * angular_velocity;
  right_wheel_target_velocity =
    linear_velocity + (WHEEL_BASE / 2.0F) * angular_velocity;

  // Keep target speeds inside the range used by the feed-forward model.
  left_wheel_target_velocity = constrain(
    left_wheel_target_velocity,
    -MAX_WHEEL_SPEED_MPS,
    MAX_WHEEL_SPEED_MPS
  );
  right_wheel_target_velocity = constrain(
    right_wheel_target_velocity,
    -MAX_WHEEL_SPEED_MPS,
    MAX_WHEEL_SPEED_MPS
  );

  last_cmd_vel_ms = millis();
  has_received_cmd_vel = true;

}

// Initialize hardware, micro-ROS transport, messages, and ROS entities once.
void setup()
{
  // Initialize motor driver and wheel encoders
  motors.begin();
  encoders.begin();

  // Ensure motors are stopped at startup
  motors.stop();
  encoders.setDirections(0, 0);

  // Initialize the MPU6050 before networking. The robot must remain stationary
  // while the gyro bias is measured.
  if (!imu.setupIMU())
  {
    imu_available = false;
  }
  else
  {
    if (imu.calibrateGyro())
    {
      imu_available = true;
    }
    else
    {
      imu_available = false;
    }
  }

  // Configure WiFi transport for micro-ROS
  set_microros_wifi_transports(
    WIFI_SSID,
    WIFI_PASSWORD,
    agent_ip,
    agent_port
  );

  delay(2000);

  // Initialize allocator
  allocator = rcl_get_default_allocator();

  // Initialize ROS support structure
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create ROS Node
  rclc_node_init_default(&node, "esp32_node", "", &support);

  initializeMessages();

  // Create subscriber for /cmd_vel
  rclc_subscription_init_best_effort(
    &subscriber,
    &node,

    // Message type
    ROSIDL_GET_MSG_TYPE_SUPPORT(
      geometry_msgs,
      msg,
      Twist
    ),

    // Topic name
    "/cmd_vel"
  );

  rclc_publisher_init_best_effort(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "/imu/data_raw"
  );

  rclc_publisher_init_best_effort(
    &temperature_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
    "/imu/temperature"
  );

  rclc_publisher_init_best_effort(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"
  );

  // Synchronize message timestamps with the ROS computer.
  rmw_uros_sync_session(1000);

  // Initializw executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Add subscriber callback to executor
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &cmd_vel_msg,
    &cmd_vel_callback,

    // Trigger callback only when new data arrives
    ON_NEW_DATA
  );

}

// Service ROS work, update sensors and motors, and enforce command timeout.
void loop()
{
  rclc_executor_spin_some(
    &executor,

    // Max processing time
    RCL_MS_TO_NS(10)
  );

  delay(10);

  // Run closed-loop wheel control before publishing its latest measurement.
  updateWheelVelocityControl();
  readAndPublishEncoders();
  readAndPublishIMU();

  // Stop motors if no cmd_vel message received within timeout period
  if (has_received_cmd_vel && millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS)
  {
    left_wheel_target_velocity = 0.0F;
    right_wheel_target_velocity = 0.0F;
    resetWheelPID(left_wheel_pid);
    resetWheelPID(right_wheel_pid);
    encoders.setDirections(0, 0);
    motors.stop();
    has_received_cmd_vel = false;
  }
}
