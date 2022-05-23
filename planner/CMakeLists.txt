cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  eigen_catkin
  catkin_simple
  mav_msgs
  mav_planning_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  mav_visualization
  roslib
)

#catkin_simple(ALL_DEPS_REQUIRED)

add_service_files(
  FILES
  GetTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp	message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(planner 
  src/planner.cpp 
  src/planner_node.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
