# bot_description

ROS 2 description package for the 2-wheel differential drive robot.

This package owns the robot model used by RViz, Gazebo, and launch files in
`bot_bringup`. It contains the URDF/xacro files, mesh assets, and the Onshape
export workflow used to regenerate the model.

## Package contents

```text
bot_description/
|-- meshes/             # STL mesh assets used by the URDF
|-- onshape/            # Onshape export config and post-processing script
|-- urdf/
|   |-- bot.urdf        # Generated base URDF from Onshape
|   |-- bot.urdf.xacro  # Main robot description entry point
|   |-- gazebo.xacro    # Gazebo materials, friction, and simulation plugins
|-- CMakeLists.txt
|-- package.xml
```

## Main description file

`urdf/bot.urdf.xacro` is the entry point for the robot description.

It currently includes:

- `urdf/bot.urdf`: the generated robot links, joints, and mesh references.
- `urdf/gazebo.xacro`: Gazebo-specific materials, friction values, differential
  drive plugin configuration, and joint state publishing.



## Build

From the workspace root:

```bash
colcon build --packages-select bot_description
source install/setup.bash
```

## Run in simulation

This package provides the description assets, but the launch flow lives in
`bot_bringup`.

The usual flow is:

```bash
colcon build
source install/setup.bash
ros2 launch bot_bringup sim.launch.py
```

## Regenerate from Onshape

The base URDF and mesh assets are generated from the files in `onshape/`.

From `src/bot_description/onshape`:

```bash
onshape-to-robot . && python3 postprocess.py
```

The post-processing script copies STL files into `meshes/`, writes the processed
URDF to `urdf/bot.urdf`, simplifies selected collision geometry, and injects the
`base_link` root link.

See `onshape/README.md` for the Onshape export notes.
