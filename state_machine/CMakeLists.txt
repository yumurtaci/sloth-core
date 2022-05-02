cmake_minimum_required(VERSION 3.0.2)
project(state_machine)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  
)

find_package(Eigen3 REQUIRED)
find_package(catkin_simple REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}

)

add_executable(state_machine 
	src/state_machine.cpp
	src/state_machine_node.cpp
	include/state_machine/state_machine.h
	)
add_dependencies(state_machine ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_machine ${catkin_LIBRARIES})

cs_install()
cs_export()
