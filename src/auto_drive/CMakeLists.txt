cmake_minimum_required(VERSION 2.8.3)
project(auto_drive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  driverless_msgs
  roscpp
  rospy
  serial
  std_msgs
  actionlib
  actionlib_msgs
)

add_action_files(
    DIRECTORY action 
    FILES DoAutoDrive.action
)

generate_messages(DEPENDENCIES actionlib_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES auto_drive
  CATKIN_DEPENDS driverless_msgs roscpp rospy serial std_msgs jsk_recognition_msgs interface message_runtime
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


aux_source_directory (src SRC_FILE)

add_executable(${PROJECT_NAME}_node ${SRC_FILE})
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES}  ${Boost_LIBRARIES} ${catkin_LIBRARIES})

