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

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#modify the include path to find the boost headers
rosbuild_add_boost_directories()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()


rosbuild_add_executable(disparity_segmentation src/disparity_segmentation.cpp)
rosbuild_link_boost(disparity_segmentation signals)
rosbuild_add_executable(template_segmentation src/template_segmentation.cpp
                                              src/image_processing.cpp)
rosbuild_link_boost(template_segmentation signals)

rosbuild_add_executable(histogram_segmentation src/histogram_segmentation.cpp
                                               src/histogram.cpp
                                               src/image_processing.cpp)

rosbuild_link_boost(histogram_segmentation signals system filesystem)

rosbuild_add_library(${PROJECT_NAME} src/image_processing.cpp src/histogram.cpp)
rosbuild_add_library(histogram src/histogram.cpp)
rosbuild_link_boost(${PROJECT_NAME} signals)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

################################
# Unit Tests
################################
# Add test cpp file
rosbuild_add_gtest(histogram_segmentation_unit_test pal_test/histogram_segmentation_unit_test.cpp TIMEOUT 60.0)
target_link_libraries(histogram_segmentation_unit_test histogram)

