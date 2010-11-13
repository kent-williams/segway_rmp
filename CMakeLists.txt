cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init(segway_apox)
rosbuild_genmsg()
rosbuild_add_library(segway_apox segway_apox.cpp) # usbcan.c serial.c)

rosbuild_add_executable(print_encoders print_encoders.cpp)
target_link_libraries(print_encoders segway_apox)

rosbuild_add_executable(time_encoders time_encoders.cpp)
target_link_libraries(time_encoders segway_apox)

rosbuild_add_executable(segway_node segway_node.cpp)
target_link_libraries(segway_node segway_apox)

