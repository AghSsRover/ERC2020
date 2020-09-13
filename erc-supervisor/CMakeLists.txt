cmake_minimum_required(VERSION 3.0.2)
project(erc_supervisor)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  move_base_msgs
  tf2_ros
  message_generation
  tf2
  tf2_geometry_msgs
  fiducial_slam
  rotator
)

add_service_files(
  FILES
  SeekOrientation.srv
  UserRotate.srv
  SetGoalsId.srv
)

add_message_files(
   FILES
   ErcGoal.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs  
  fiducial_slam
  rotator
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES erc_supervisor
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

set(SOURCES
src/erc_supervisor_main.cpp
src/ErcSupervisor.cpp
)


add_executable(${PROJECT_NAME}_node ${SOURCES})

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME ${PROJECT_NAME} PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY
cfg
launch
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
) 