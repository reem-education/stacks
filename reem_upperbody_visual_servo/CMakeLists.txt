cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# Migration to Hydro: uncomment the following lines and make sure to install visp in /usr
# Get ViSP specific definitions
# ADD_DEFINITIONS( -DUNIX )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#rosbuild_add_executable(old_upperbody src/old_reem_upperbody_visual_servo.cpp
#                                      src/utility_functions.cpp
#                                      src/conversions.cpp
#                                      src/tf_utils.cpp
#                                      src/move_parts.cpp
#                                      src/secondary_task.cpp)

#target_link_libraries(old_upperbody yaml-cpp
#                      #visp      #migration to Hydro: uncomment this line
#                     )


rosbuild_add_executable(upperbody src/reem_upperbody_visual_servo.cpp
                                  src/utility_functions.cpp
                                  src/conversions.cpp
                                  src/tf_utils.cpp
                                  src/kinematic_chain.cpp
                                  src/hand_task.cpp
                                  src/head_task.cpp
                                  src/secondary_task.cpp
                                  src/move_parts.cpp)

target_link_libraries(upperbody yaml-cpp
                      #visp      #migration to Hydro: uncomment this line
                     )

rosbuild_add_executable(learn_desired_state src/learn_desired_state.cpp
                                            src/utility_functions.cpp
                                            src/conversions.cpp
                                            src/tf_utils.cpp)

target_link_libraries(learn_desired_state yaml-cpp
                      #visp      #migration to Hydro: uncomment this line
                     )


