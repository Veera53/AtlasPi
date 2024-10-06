cmake_minimum_required(VERSION 3.0.2)
project(ekf_localization)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  eigen_conversions
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(ekf_localization_node src/ekf_localization_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(ekf_localization_node
  ${catkin_LIBRARIES}
)
