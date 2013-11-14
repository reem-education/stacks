cmake_minimum_required(VERSION 2.8.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
if (NOT OSRF_COMMON_VERSION)
  message("WOAH THERE PARTNER. no version supplied. don't install this.")
  set(OSRF_COMMON_VERSION bogus) # only used for non-install builds
endif()
set(PKG_NAME osrf_msgs)
set(ENV{ROS_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_genmsg()
set(ROS_PKG_INSTALL_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/osrf-common-${OSRF_COMMON_VERSION}/ros/${PKG_NAME})

install(FILES manifest.xml 
        DESTINATION ${ROS_PKG_INSTALL_SHARE_PATH})
install(DIRECTORY msg msg_gen src DESTINATION ${ROS_PKG_INSTALL_SHARE_PATH} PATTERN ".*" EXCLUDE)
