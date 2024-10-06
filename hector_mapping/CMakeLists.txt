cmake_minimum_required(VERSION 3.0.2)
project(hector_mapping)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(hector_mapping_node src/hector_mapping_node.cpp)

target_link_libraries(hector_mapping_node
  ${catkin_LIBRARIES}
)
