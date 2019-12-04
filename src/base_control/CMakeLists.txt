cmake_minimum_required(VERSION 2.8.3)
project(base_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
  driverless_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES base_control
#  CATKIN_DEPENDS roscpp serial std_msgs driverless_msgs 
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/. SRC)
add_executable(${PROJECT_NAME}_node ${SRC})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

