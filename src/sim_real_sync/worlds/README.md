Place Gazebo world files for this package in this directory.

Default launch hook:
- `gazebo_world`: `$(find sim_real_sync)/worlds/demo.world`

You can switch worlds at launch time without changing the main bringup:
- `roslaunch sim_real_sync sim_real_sync.launch gazebo_world:=<path-to-world>`
