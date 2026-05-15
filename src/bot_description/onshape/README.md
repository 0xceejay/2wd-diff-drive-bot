# Onshape export

This folder contains the files used to regenerate the robot's base URDF and
mesh assets from Onshape.

CAD assembly: [Onshape](https://cad.onshape.com/documents/292c33a7f0f854a3cdb21f44/w/3cfe75fb330dfc627e8355b6/e/a29efea0b989aa0e985d95df)

The generated files are copied into the parent `bot_description` package:

- `../urdf/bot.urdf`
- `../meshes/*.stl`

## Files

- `config.json`: configuration used by `onshape-to-robot`.
- `postprocess.py`: cleanup script that adapts the generated model for this ROS
  2 package and Gazebo simulation.

## Export workflow

Run this command from this folder:

```bash
onshape-to-robot . && python3 postprocess.py
```

`onshape-to-robot` reads `config.json`, exports the `robot` assembly from
Onshape, and creates a temporary `robot.urdf` plus an `assets/` folder.

`postprocess.py` then:

- writes the processed URDF to `../urdf/bot.urdf`
- copies exported STL files into `../meshes/`
- changes mesh paths from `assets/` to `meshes/`
- adds `base_link` as the root link with a fixed joint to `chassis`
- removes collision geometry from small/internal parts
- replaces selected collision meshes with simple box, cylinder, or sphere shapes
- adjusts wheel collision orientation and offsets
- removes the temporary `robot.urdf` and `assets/` folder

## Notes

This workflow overwrites the generated URDF and refreshes mesh files in the
package. After regenerating, check the robot in simulation with:

```bash
colcon build
source install/setup.bash
ros2 launch bot_bringup sim.launch.py
```
