# 2WD Differential Drive Bot

A work-in-progress 2-wheel differential drive robot built with ROS 2, Gazebo,
micro-ROS, and ESP32 firmware.

The project currently includes the robot description, Gazebo simulation launch
files, joystick teleoperation, a micro-ROS agent launch flow, and PlatformIO
firmware for the ESP32 controller.

## Repository layout

```text
2wd-diff-drive-bot/
|-- firmware/
|   |-- bot_firmware/        # ESP32 PlatformIO firmware
|-- src/
|   |-- bot_bringup/         # ROS 2 launch files and teleop config
|   |-- bot_description/     # URDF, xacro files, meshes, and Onshape export
|-- LICENSE
|-- README.md
```

## Main parts

| Path | Purpose |
| --- | --- |
| `src/bot_description` | Robot model, URDF/xacro files, Gazebo tags, meshes, and Onshape export workflow. |
| `src/bot_bringup` | Current launch flows for joystick teleop, micro-ROS, and Gazebo simulation. |
| `firmware/bot_firmware` | ESP32 firmware that connects to ROS 2 through micro-ROS over Wi-Fi. |


## Requirements

- ROS 2 Jazzy
- Gazebo / `ros_gz_sim` and `ros_gz_bridge`
- `xacro`
- `robot_state_publisher`
- `joy`
- `teleop_twist_joy`
- `micro_ros_agent`
- PlatformIO for the ESP32 firmware

## Build the ROS 2 workspace

From the repository root:

```bash
colcon build
source install/setup.bash
```

To build one package:

```bash
colcon build --packages-select bot_description
colcon build --packages-select bot_bringup
```

## Run simulation

```bash
source install/setup.bash
ros2 launch bot_bringup sim.launch.py
```

This starts Gazebo, publishes the robot description, spawns the robot, and
bridges the main Gazebo topics to ROS 2.

## Run the physical robot flow

Start joystick teleop and the micro-ROS agent:

```bash
source install/setup.bash
ros2 launch bot_bringup robot.launch.py
```

The ESP32 firmware must be flashed and configured with the ROS 2 computer's IP
address in `firmware/bot_firmware/include/wifi_secrets.hpp`.

## Firmware

From the repository root:

```bash
pio run -d firmware/bot_firmware
pio run -d firmware/bot_firmware --target upload
```

See `firmware/bot_firmware/README.md` for firmware setup, Wi-Fi configuration,
and micro-ROS notes.

## Documentation

- `src/bot_description/README.md`: robot description package notes.
- `src/bot_description/onshape/README.md`: Onshape export and post-processing
  workflow.
- `src/bot_bringup/README.md`: launch files and startup flows.
- `firmware/bot_firmware/README.md`: ESP32 firmware setup and commands.

## License

This project is licensed under the MIT License. See `LICENSE`.
