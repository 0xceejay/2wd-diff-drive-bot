# bot_bringup

ROS 2 launch package for starting the 2-wheel differential drive robot.

For now, this package ties together joystick teleoperation, the micro-ROS agent,
and the Gazebo simulation launch flow. As the robot software grows, this package
will also become the place for additional launch files and startup flows. The
robot description itself lives in `bot_description`, while the ESP32 firmware
lives in `firmware/bot_firmware`.

## Package contents

```text
bot_bringup/
|-- config/
|   `-- teleop.config.yaml
|-- launch/
|   |-- micro_ros.launch.py
|   |-- robot.launch.py
|   |-- sim.launch.py
|   `-- teleop.launch.py
|-- CMakeLists.txt
|-- package.xml
`-- README.md
```

## Launch files

Current launch files:

| File | Purpose |
| --- | --- |
| `robot.launch.py` | Starts joystick teleop and the micro-ROS agent for the physical robot. |
| `teleop.launch.py` | Starts `joy_node` and `teleop_twist_joy` for joystick control. |
| `micro_ros.launch.py` | Starts the micro-ROS agent using UDP on port `8888`. |
| `sim.launch.py` | Starts Gazebo, publishes the robot description, spawns the robot, and bridges Gazebo topics to ROS 2. |

## Build

From the workspace root:

```bash
colcon build --packages-select bot_bringup
source install/setup.bash
```

## Physical robot bringup

Use this when the ESP32 firmware is flashed and connected to the same network as
the ROS 2 computer.

```bash
ros2 launch bot_bringup robot.launch.py
```

This starts:

- joystick teleoperation
- micro-ROS agent on UDP port `8888`

The ESP32 firmware should be configured with the IP address of this ROS 2
computer in `firmware/bot_firmware/include/wifi_secrets.hpp`.

## micro-ROS agent only

Use this if you want to start only the agent:

```bash
ros2 launch bot_bringup micro_ros.launch.py
```

This is useful when testing the ESP32 firmware separately from joystick teleop.

## Joystick teleop only

```bash
ros2 launch bot_bringup teleop.launch.py
```

Default launch arguments:

| Argument | Default | Purpose |
| --- | --- | --- |
| `joy_vel` | `cmd_vel` | Output velocity topic. |
| `joy_config` | `ps3` | Teleop config name used by `teleop_twist_joy`. |
| `joy_dev` | `0` | Joystick device ID. |
| `publish_stamped_twist` | `false` | Publish plain `Twist` instead of stamped `TwistStamped`. |

The local joystick config is in `config/teleop.config.yaml`.

Current control mapping:

- Linear axis: `1`
- Angular axis: `0`
- Enable button: `4`
- Turbo button: `10`

## Simulation

```bash
ros2 launch bot_bringup sim.launch.py
```

This launch file:

- starts Gazebo with `empty.sdf`
- expands `bot_description/urdf/bot.urdf.xacro`
- starts `robot_state_publisher`
- spawns the robot as `my_bot`
- bridges `/cmd_vel`, `/odom`, and `/tf` between ROS 2 and Gazebo

## Related packages

- `bot_description`: robot URDF, xacro files, meshes, and simulation tags.
- `firmware/bot_firmware`: ESP32 PlatformIO firmware that subscribes to
  `/cmd_vel` through micro-ROS.
