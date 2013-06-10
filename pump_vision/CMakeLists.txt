cmake_minimum_required(VERSION 2.8.3)
project(pump_vision)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp image_transport cv_bridge sensor_msgs)
find_package(OpenCV REQUIRED)

add_executable(pump_pose_detection src/pump_pose_detection.cpp)

target_link_libraries(pump_pose_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})

catkin_package(CATKIN_DEPENDS roscpp std_msgs sensor_msgs
  DEPENDS OpenCV image_transport cv_bridge)
