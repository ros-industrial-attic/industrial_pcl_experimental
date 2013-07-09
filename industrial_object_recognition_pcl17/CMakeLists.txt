cmake_minimum_required(VERSION 2.8.3)
project(industrial_object_recognition_pcl17)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp industrial_utils visualization_msgs tf tf_conversions message_runtime genmsg pcl pcl_ros)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread signals system)
#find_package(PCL REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(/usr/include/vtk-5.8)
include_directories(/home/cgomez/ros/groovy/pcl_ws/install_isolated/include/pcl-1.7)
#find_library(REQUIRED)
#find_package(pcl REQUIRED)
link_directories(/home/cgomez/ros/groovy/pcl_ws/install_isolated/lib)
#find_library(PCL_LIBRARY pcl /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib)
add_library(pcllib_apps SHARED IMPORTED)
add_library(pcllib_search SHARED IMPORTED)
add_library(pcllib_common SHARED IMPORTED)
add_library(pcllib_recognition SHARED IMPORTED)
add_library(pcllib_features SHARED IMPORTED)
add_library(pcllib_filters SHARED IMPORTED)
add_library(pcllib_sampleconsensus SHARED IMPORTED)
set_target_properties(pcllib_apps PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_apps.so.1.7.0)
set_target_properties(pcllib_search PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_search.so.1.7.0)
set_target_properties(pcllib_common PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_common.so.1.7.0)
set_target_properties(pcllib_recognition PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_recognition.so.1.7.0)
set_target_properties(pcllib_features PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_features.so.1.7.0)
set_target_properties(pcllib_filters PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_filters.so.1.7.0)
set_target_properties(pcllib_sampleconsensus PROPERTIES IMPORTED_LOCATION /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib/libpcl_sample_consensus.so.1.7.0)

# Declare the message files to be built
add_message_files(FILES
    pose.msg
  )

# Declare the service files to be built
add_service_files(FILES
    object_recognition.srv 
  )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES std_msgs sensor_msgs visualization_msgs
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_executable(recognition_node src/recognition_node.cpp)
add_executable(recognition_service src/recognition_service.cpp)
#add_executable(training_node src/generate_training_data.cpp)
#add_executable(data_processing_node src/process_faro_data.cpp)
target_link_libraries(recognition_node libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkIO.so libvtkGraphics.so)
#target_link_libraries(recognition_node librostime.so)
target_link_libraries(recognition_node pcllib_apps pcllib_common pcllib_search pcllib_recognition pcllib_features pcllib_filters pcllib_sampleconsensus)#${PCL_LIBRARY}
#target_link_libraries(recognition_node ${PCL_LIBRARIES})
target_link_libraries(recognition_node ${catkin_LIBRARIES})
#target_link_libraries(recognition_node libpcl_apps.so.1.7.0 libpcl_common.so.1.7.0 libpcl_features.so.1.7.0 libpcl_filters.so.1.7.0 libpcl_io.so.1.7.0)
target_link_libraries(recognition_service ${catkin_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkIO.so libvtkGraphics.so)
target_link_libraries(recognition_service pcllib_apps pcllib_common pcllib_search pcllib_recognition pcllib_features pcllib_filters pcllib_sampleconsensus)#${PCL_LIBRARY}
#target_link_libraries(training_node ${PCL_LIBRARIES} libvtkCommon.so libvtkFiltering.so libvtkRendering.so libvtkIO.so libvtkGraphics.so)
catkin_package(
  LIBRARIES industrial_object_recognition_pcl17 /home/cgomez/ros/groovy/pcl_ws/install_isolated/lib
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs genmsg pcl pcl_ros tf tf_conversions
  DEPENDS industrial_utils) 
