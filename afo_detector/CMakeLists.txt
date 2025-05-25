cmake_minimum_required(VERSION 3.0.2)
project(afo_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(afo_detector src/main.cpp src/jsoncpp.cpp)
target_link_libraries(afo_detector ${catkin_LIBRARIES})
