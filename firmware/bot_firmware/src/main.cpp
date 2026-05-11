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

  // Print results
  Serial.print("Left Wheel Velocity: ");
  Serial.print(left_wheel_velocity);
  Serial.print(" | Right Wheel Velocity: ");
  Serial.println(right_wheel_velocity);
}

// SETUP
void setup()
{
  // Start serial communication
  Serial.begin(115200);
  delay(2000);

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
  rclc_subscription_init_default(
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
}

