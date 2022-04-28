cmake_minimum_required(VERSION 3.0.2)
project(offboard_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  mavros
  mavros_extras
  mavros_msgs
  mavlink
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
add_executable(${PROJECT_NAME} src/offb_node.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )


