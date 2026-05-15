# bot_firmware

PlatformIO firmware for the ESP32 controller on the 2-wheel differential drive
robot.

The firmware uses micro-ROS over Wi-Fi to connect the ESP32 to the ROS 2
computer. It subscribes to `/cmd_vel`, reads incoming
`geometry_msgs/msg/Twist` messages, and calculates left/right wheel velocity
targets from the robot geometry.

## Hardware and framework

- Board: ESP32 DOIT DevKit V1
- Framework: Arduino
- Build system: PlatformIO
- micro-ROS distro: Jazzy
- micro-ROS transport: Wi-Fi
- Serial monitor speed: `115200`
- Upload speed: `921600`
- micro-ROS agent UDP port: `8888`

These settings live in `platformio.ini`.

## Project layout

```text
bot_firmware/
|-- include/
|   |-- wifi_secrets.hpp  # Local Wi-Fi and agent IP settings
|   `-- README
|-- lib/                  # Optional private PlatformIO libraries
|-- src/
|   `-- main.cpp          # ESP32 micro-ROS node
|-- test/                 # PlatformIO tests
|-- platformio.ini
`-- README.md
```

## Local Wi-Fi settings

`src/main.cpp` expects a local header at:

```text
include/wifi_secrets.hpp
```

It should define:

```cpp
#define WIFI_SSID "your_wifi_name"
#define WIFI_PASSWORD "your_wifi_password"

#define AGENT_IP_1 192
#define AGENT_IP_2 168
#define AGENT_IP_3 1
#define AGENT_IP_4 10
```

The `AGENT_IP_*` values should match the IP address of the computer running the
micro-ROS agent.

Keep this file local because it contains Wi-Fi credentials.

## Build

From this folder:

```bash
pio run
```

Or from the workspace root:

```bash
pio run -d firmware/bot_firmware
```

## Upload

From this folder:

```bash
pio run --target upload
```

Or from the workspace root:

```bash
pio run -d firmware/bot_firmware --target upload
```

## Serial monitor

```bash
pio device monitor
```

The firmware prints the calculated left and right wheel velocities when it
receives `/cmd_vel` messages.

## Run with ROS 2

Start the micro-ROS agent on the ROS 2 computer:

```bash
ros2 launch bot_bringup micro_ros.launch.py
```

That launch file starts the agent with UDP on port `8888`, matching the firmware.

<!-- To start the robot bringup flow with teleop and the micro-ROS agent:

```bash
ros2 launch bot_bringup robot.launch.py
``` -->

## Subscribed topic

| Topic | Type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Command linear and angular velocity |

The current firmware computes wheel velocity targets using:

```text
left = linear.x - (wheel_base / 2) * angular.z
right = linear.x + (wheel_base / 2) * angular.z
```

Current geometry constants:

- Wheel radius: `0.033 m`
- Wheel base: `0.1443 m`
