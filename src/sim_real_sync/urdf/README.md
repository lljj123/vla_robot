Place the robot URDF or Xacro files in this directory.

Default launch parameters:
- `urdf_dir`: `$(find sim_real_sync)/urdf`
- `urdf_file`: `robot.urdf.xacro`
- `robot_description_param`: `/robot_description`

The launch file now loads the selected URDF/Xacro into `robot_description`
for RViz and `robot_state_publisher`.

RViz parameter hooks reserved in launch:
- `rviz_camera_topic`
- `rviz_imu_topic`
- `rviz_lidar_topic`
- `rviz_arm_joint_states_topic`
