cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

if(USE_ROSBUILD)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  # Generate Actions
  rosbuild_find_ros_package(actionlib_msgs)
  include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
  genaction()

  rosbuild_init()

  set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

  rosbuild_genmsg()
  rosbuild_gensrv()
else()
  find_package(catkin REQUIRED actionlib_msgs geometry_msgs message_generation std_msgs trajectory_msgs)

  add_action_files(DIRECTORY action FILES
    FollowJointTrajectory.action
    GripperCommand.action
    JointTrajectory.action
    PointHead.action
    SingleJointPosition.action)

  add_message_files(DIRECTORY msg FILES
    GripperCommand.msg
    JointControllerState.msg
    JointTolerance.msg
    JointTrajectoryControllerState.msg)

  add_service_files(DIRECTORY srv FILES
    QueryCalibrationState.srv
    QueryTrajectoryState.srv)

  generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs std_msgs trajectory_msgs)

  catkin_package(CATKIN_DEPENDS actionlib_msgs geometry_msgs message_runtime std_msgs trajectory_msgs)
endif()