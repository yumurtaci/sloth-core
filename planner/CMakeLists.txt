cmake_minimum_required(VERSION 3.0.2)
project(planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
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

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_executable(planner
        src/planner.cpp
        src/planner_node.cpp
        include/planner/planner.h
        )

cs_install()
cs_export()
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp	
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


