#!/bin/bash

# Unload curernt arm controllers for tiago dual and replace them for joint_group_ff_controllers/JointGroupEffortFFController

# Upload controllers parameters on server
rosparam load $(rospack find joint_group_ff_controllers)/examples/ur5_example_config.yaml

# Stop and unload default tiago dual controller
rosrun controller_manager controller_manager stop left_arm/scaled_vel_joint_traj_controller right_arm/scaled_vel_joint_traj_controller
rosrun controller_manager controller_manager unload left_arm/scaled_vel_joint_traj_controller right_arm/scaled_vel_joint_traj_controller

# Stop and unload potential previous joint_group_ff_controllers/JointGroupEffortFFController already started
rosrun controller_manager controller_manager stop left_arm/ff_controller right_arm/ff_controller
rosrun controller_manager controller_manager unload left_arm/ff_controller right_arm/ff_controller

# Load and run new joint_group_ff_controllers/JointGroupEffortFFController controlles
rosrun controller_manager controller_manager spawn left_arm/ff_controller right_arm/ff_controller