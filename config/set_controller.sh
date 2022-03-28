#!/bin/bash

cat ~/tiago_public_ws/src/joint_group_ff_controllers/config/controller.yaml
rosparam load ~/tiago_public_ws/src/joint_group_ff_controllers/config/controller.yaml
rosrun controller_manager controller_manager reload-libraries
rosrun controller_manager controller_manager stop arm_left_controller arm_right_controller
rosrun controller_manager controller_manager unload arm_left_controller arm_right_controller
rosrun controller_manager controller_manager spawn arm_left_controller arm_right_controller