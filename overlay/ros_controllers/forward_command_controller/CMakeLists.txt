cmake_minimum_required(VERSION 2.8.3)
project(forward_command_controller)

if(USE_ROSBUILD)

  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

  rosbuild_init()

else()

  # Load catkin and all dependencies required for this package
  find_package(catkin REQUIRED COMPONENTS controller_interface hardware_interface std_msgs)

  # Declare catkin package
  catkin_package(
    CATKIN_DEPENDS controller_interface hardware_interface std_msgs
    INCLUDE_DIRS include
    )

  # Install
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

endif()
