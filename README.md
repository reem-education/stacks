
Introduction
============


REEM, the humanoid service robot created by PAL Robotics, can be now used for educational purposes in simulation. These packages show different capabilities of the robot.

Requirements
============

* Ubuntu 12.04
* ROS Fuerte
* Gazebo 1.8.6 standalone

Installation procedure
======================

rosinstall . https://raw.github.com/reem-education/stacks/fuerte/reem-education-ros-fuerte-gazebo-1.8.6-standalone.rosinstall

Setup
=====

Append the following text in your ~/.bashrc file:

	. /opt/ros/fuerte/setup.bash

	. /usr/share/gazebo-1.8/setup.sh

	export ROS_PACKAGE_PATH=$HOME/reem-education/stacks:/opt/ros/fuerte/share:/opt/ros/fuerte/stacks
	export GAZEBO_PLUGIN_PATH=`rospack find ros_control_gazebo_plugin`/lib:$GAZEBO_PLUGIN_PATH
	export GAZEBO_PLUGIN_PATH=`rospack find atlas_msgs`/lib:$GAZEBO_PLUGIN_PATH
	export GAZEBO_PLUGIN_PATH=`rospack find pal_gazebo_plugins`/lib:$GAZEBO_PLUGIN_PATH
	export GAZEBO_MODEL_PATH=`rospack find reem_gazebo`/models:$GAZEBO_MODEL_PATH
	export GAZEBO_RESOURCE_PATH=`rospack find reem_gazebo`/reem_gazebo/worlds:$GAZEBO_RESOURCE_PATH
