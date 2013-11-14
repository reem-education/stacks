#!/bin/bash

./stop_perception.sh

#load movements in rosparam
rosparam load `rospack find reem_upperbody_visual_servo`/config/reem_poses.yaml

#moving robot to initial pose:

rosrun reem_tutorials reach_pose initial_grasp_pose_step1 

#sleep 6

rosrun reem_tutorials reach_pose initial_grasp_pose_step2

#undistort image:

ROS_NAMESPACE=/stereo/left rosrun image_proc image_proc image_raw:=image &

#launch ARuco marker detector:

roslaunch aruco_ros double.launch &


