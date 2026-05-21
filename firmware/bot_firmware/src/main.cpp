#include <Arduino.h>
#include <micro_ros_platformio.h>

// Core ROS 2 C libraries
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Message type for /cmd_vel
#include <geometry_msgs/msg/twist.h>

// WIFI credentials
#include "wifi_secrets.hpp"

// IP addresso of laptop running micro-ROS agent
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
rcl_node_t node;
geometry_msgs__msg__Twist msg;

// Robot geometry
const float WHEEL_RADIUS = 0.033;
const float WHEEL_BASE = 0.1443;

// Velocity variables
float left_wheel_velocity = 0.0;
float right_wheel_velocity = 0.0;

// TB6612FNG motor pins
const int PWMA = 25;
const int AIN1 = 26;
const int AIN2 = 27;

const int PWMB = 13;
const int BIN1 = 33;
const int BIN2 = 32;

const int STBY = 14;

// PWM settings
const int PWM_FREQ = 20000;

// PWM resolution in bits
// 8 bits (values from 0 to 255)
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_A = 0;
const int PWM_CHANNEL_B = 1;

// Speed limit
const int MAX_PWM = 80;
const int MIN_MOVING_PWM = 30;
const unsigned long CMD_VEL_TIMEOUT_MS = 500;

// Motor trim
// Use to reduce speed if one motor moves faster than the other
const float LEFT_MOTOR_TRIM = 0.625;
const float RIGHT_MOTOR_TRIM = 1.00;

// Timestamp of last received /cmd_vel message
unsigned long last_cmd_vel_ms = 0;

// Flag to track if we've received at least one /cmd_vel message
bool has_received_cmd_vel = false;

// Clamp PWM values to at least MIN_MOVING_PWM in magnitude for small inputs.
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

// Motor function to initialize motor driver
void setupMotors()
{
  // Configure direction pins as outputs
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Enable motor driver
  digitalWrite(STBY, HIGH);

  // Configure ESP32 PWM channels
  ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);

  // Attach PWM channels to physical pins
  ledcAttachPin(PWMA, PWM_CHANNEL_A);
  ledcAttachPin(PWMB, PWM_CHANNEL_B);
}

// CONTROL LEFT MOTOR
// pwm range: -255 -> full reverse; 0 -> stop; 255 -> full forward
void setLeftMotor(int pwm)
{
  // Prevent invalid PWM values
  pwm = constrain(pwm, -255, 255);

  // Forward
  if (pwm > 0)
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }

  // Reverse
  else if (pwm < 0)
  {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }

  // Stop
  else
  {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);
  }

  // Apply PWM speed
  ledcWrite(PWM_CHANNEL_A, abs(pwm));
}

void setRightMotor(int pwm)
{
  // Prevent invalid PWM values
  pwm = constrain(pwm, -255, 255);

  // Forward
  if (pwm > 0)
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }

  // Reverse
  else if (pwm < 0)
  {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  // Stop
  else
  {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
  }

  // Apply PWM speed
  ledcWrite(PWM_CHANNEL_B, abs(pwm));
}

// STOP BOTH MOTORS
void stopMotors()
{
  setLeftMotor(0);
  setRightMotor(0);
}

// CALLBACK FUNC: runs automatically whenever a new /cmd_vel message is received
void cmd_vel_callback(const void * msgin)
{
  // Cast incoming generic pointer to Twist message type
  const geometry_msgs__msg__Twist * twist_msg =
    (const geometry_msgs__msg__Twist *)msgin;

  float linear_velocity = twist_msg->linear.x;
  float angular_velocity = twist_msg->angular.z;

  left_wheel_velocity = linear_velocity - (WHEEL_BASE / 2.0) * angular_velocity;
  right_wheel_velocity = linear_velocity + (WHEEL_BASE / 2.0) * angular_velocity;

  float max_wheel_speed = 0.5; // m/s

  // Scale wheel velocity to PWM range
  int left_pwm =
    (left_wheel_velocity / max_wheel_speed) * MAX_PWM * LEFT_MOTOR_TRIM;
  int right_pwm =
    (right_wheel_velocity / max_wheel_speed) * MAX_PWM * RIGHT_MOTOR_TRIM;

  // Prevent unsafe PWM values
  left_pwm = constrain(left_pwm, -MAX_PWM, MAX_PWM);
  right_pwm = constrain(right_pwm, -MAX_PWM, MAX_PWM);

  // Pure turns give very small PWM values, too small to move the motors.
  left_pwm = applyMinimumMovingPwm(left_pwm);
  right_pwm = applyMinimumMovingPwm(right_pwm);

  // Send commands to motors
  setLeftMotor(left_pwm);
  setRightMotor(right_pwm);

  last_cmd_vel_ms = millis();
  has_received_cmd_vel = true;

  // Print results
  Serial.print("Linear: ");
  Serial.print(linear_velocity);
  Serial.print(" | Angular: ");
  Serial.print(angular_velocity);
  Serial.print(" | Left PWM: ");
  Serial.print(left_pwm);
  Serial.print(" | Right PWM: ");
  Serial.println(right_pwm);
}

// SETUP
void setup()
{
  // Start serial communication
  Serial.begin(115200);
  delay(2000);

  // Initialize motor driver
  setupMotors();

  // Ensure motors are stopped at startup
  stopMotors();

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

  // Initializw executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Add subscriber callback to executor
  rclc_executor_add_subscription(
    &executor,
    &subscriber,
    &msg,
    &cmd_vel_callback,

    // Trigger callback only when new data arrives
    ON_NEW_DATA
  );

  Serial.println("Ready to receive cmd_vel");
}

void loop()
{
  rclc_executor_spin_some(
    &executor,

    // Max processing time
    RCL_MS_TO_NS(10)
  );

  delay(10);

  // Stop motors if no cmd_vel message received within timeout period
  if (has_received_cmd_vel && millis() - last_cmd_vel_ms > CMD_VEL_TIMEOUT_MS)
  {
    stopMotors();
    has_received_cmd_vel = false;
    Serial.println("cmd_vel timeout; motors stopped");
  }
}
