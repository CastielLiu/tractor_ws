cmake_minimum_required(VERSION 2.8.3)
project(record_path)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  gps_msgs
  serial
  interface
  auto_drive
)

catkin_package(
  #INCLUDE_DIRS include
#  LIBRARIES record_gps_data
  CATKIN_DEPENDS roscpp rospy std_msgs interface
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/record_path.cpp )
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node   ${catkin_LIBRARIES})


