cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_executable(reach_pose                           src/reach_pose.cpp)
rosbuild_add_executable(move_left_arm_joint_goal             src/move_left_arm_joint_goal.cpp)
rosbuild_add_executable(move_right_arm_torso_pose_goal       src/move_right_arm_torso_pose_goal.cpp)
rosbuild_add_executable(look_to_point                        src/look_to_point.cpp)
