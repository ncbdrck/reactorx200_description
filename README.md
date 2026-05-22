# reactorx200_description

Trossen ReactorX-200 sim extras for the `rl_environments` stack:

- Wraps the upstream `interbotix_xsarm_descriptions/urdf/rx200.urdf.xacro`.
- Adds a head-mount **Kinect v2** (and ZED2 variant) at a fixed pose.
- Brings up **ros_control** with PID gains tuned for the 5-DOF arm
  (`waist, shoulder, elbow, wrist_angle, wrist_rotate`) + 2-finger
  prismatic gripper.
- Ships local table model under `models/` so the RX200 env spawns the
  same workspace as VX300S / NED2 sims.

## Prerequisites

- ROS Noetic
- `interbotix_xsarm_descriptions`, `interbotix_xsarm_moveit_interface`,
  `interbotix_xsarm_gazebo` (Trossen — installed via `xsarm_amd64_install.sh`)
- `common_sensors` (Kinect v2 / ZED2 xacros)

## Verify the sim works

```bash
roscore                                                       # term 1
roslaunch reactorx200_description reactorx200_gazebo.launch   # term 2
```

Expected:
- Gazebo opens with the table + arm.
- `rostopic echo /rx200/joint_states` publishes the arm + gripper
  joints at ~50 Hz.
- `rostopic list | grep /arm_controller` shows command + state topics.

## Why this exists

Separate description package keeps URDF / launch files reusable for
MoveIt / RViz demos independent of the RL stack. Same shape as
`viperx300s_description` (VX300S) and `niryo_ned2_description_extras`
(NED2).

## Contact

[j.kapukotuwa@research.ait.ie](mailto:j.kapukotuwa@research.ait.ie)
