cmake_minimum_required(VERSION 2.8.3)
project(power_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    serial
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(power_control src/power_control.cpp)

target_link_libraries(power_control
  ${catkin_LIBRARIES}
)

