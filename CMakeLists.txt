cmake_minimum_required(VERSION 3.0.2)
project(motion_distortion_compensator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mdistortion_compensator_node
  src/main.cpp
  src/mdistortion_compensator_node.cpp
)
add_dependencies(mdistortion_compensator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mdistortion_compensator_node ${catkin_LIBRARIES})

install(TARGETS mdistortion_compensator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
