cmake_minimum_required(VERSION VERSION 2.8.3)
project(industrial_pcl_filters)

find_package(catkin REQUIRED COMPONENTS roscpp dynamic_reconfigure std_msgs pcl pcl_ros nodelet tf tf_conversions sensor_msgs)
find_package(PCL)

# Enable dynamic_reconfigure
generate_dynamic_reconfigure_options(cfg/ConcatenateMLS.cfg)
#include_directories(include "/opt/ros/groovy/include")
include_directories(include cfg/cpp)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})

add_library(industrial_pcl_filters src/concatenate_mls.cpp
	src/concatenate_mls_nodelet.cpp)

#add_executable(concatenate_average src/nodes/concatenate_mls_test.cpp)

#add_dependencies(concatenate_average ${industrial_pcl_filters_EXPORTED_TARGETS})
add_dependencies(industrial_pcl_filters ${industrial_pcl_filters_EXPORTED_TARGETS})

target_link_libraries(industrial_pcl_filters ${catkin_LIBRARIES})
#target_link_libraries(concatenate_average industrial_pcl_filters)
#target_link_libraries(concatenate_average )

catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp dynamic_reconfigure std_msgs pcl pcl_ros nodelet tf tf_conversions sensor_msgs)

install(FILES nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
