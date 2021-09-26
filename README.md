# Inverse kinematics for a simple biped

## This package is under heavy development, and is not ready for use.

[![Build and Test (foxy)](../../actions/workflows/build_and_test_foxy.yaml/badge.svg)](../../actions/workflows/build_and_test_foxy.yaml)
[![Build and Test (galactic)](../../actions/workflows/build_and_test_galactic.yaml/badge.svg)](../../actions/workflows/build_and_test_galactic.yaml)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg)](../../actions/workflows/build_and_test_rolling.yaml)

This package uses just the IK from [Rhoban/IKWalk](https://github.com/Rhoban/IKWalk), and wraps it inside a ROS node. Humanoids with the following kinematic model can use this package.

![](https://github.com/Rhoban/IKWalk/blob/master/Docs/humanoid.png?raw=true)

## Instructions

Run walk:
```
run walk walk --ros-args -p ankle_z:=-0.26 -p period:=0.25 -p foot_lift_amp:=0.010
```

Run biped_ik_simple:
```
ros2 run biped_ik_simple biped_ik_simple
```

Launch boldbot bringup:
```
ros2 launch boldbot_sim boldbot_sim_bringup.launch.py
```

Launch rviz2:
```
rviz2
```

Run twist teleop:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=target
```
