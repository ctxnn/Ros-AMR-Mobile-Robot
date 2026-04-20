# AMR-FE01

## Gazebo Resource Path

The `world.sdf` references meshes from the package's `meshes/` directory.  
The launch files automatically set `GZ_SIM_RESOURCE_PATH` to the package share directory, so mesh paths in the SDF are resolved automatically.

If you need to run Gazebo manually outside the launch files, set the resource path like so:

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(ros2 pkg prefix amr_fe01)/share/amr_fe01
```