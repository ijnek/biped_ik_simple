# Inverse kinematics for a simple biped

## This package is under heavy development, and is not ready for use.

[![Build and Test (humble)](../../actions/workflows/build_and_test_humble.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_humble.yaml?query=branch:rolling)
[![Build and Test (iron)](../../actions/workflows/build_and_test_iron.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_iron.yaml?query=branch:rolling)
[![Build and Test (rolling)](../../actions/workflows/build_and_test_rolling.yaml/badge.svg?branch=rolling)](../../actions/workflows/build_and_test_rolling.yaml?query=branch:rolling)

This package uses just the IK from [Rhoban/IKWalk](https://github.com/Rhoban/IKWalk), and wraps it inside a ROS node. Humanoids with the following kinematic model can use this package.

![](https://github.com/Rhoban/IKWalk/blob/master/Docs/humanoid.png?raw=true)

## Tutorial

First install the packages for the tutorial. In your workspace, run:

```
git clone git@github.com:ijnek/biped_ik_simple.git src/biped_ik_simple --recursive
vcs import src < src/biped_ik_simple/dependencies.repos --recursive
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

Run walk:
```
ros2 run walk walk --ros-args -p sole_z:=-0.26 -p period:=0.25 -p foot_lift_amp:=0.010
```

Run biped_ik_simple:
```
ros2 run biped_ik_simple biped_ik_simple
```

Launch boldbot bringup (https://gitlab.com/boldhearts/ros2_boldbot):
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
