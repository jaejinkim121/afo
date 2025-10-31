cmake_minimum_required(VERSION 3.0.2)
project(afo_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  highs REQUIRED
)

catkin_package(
)
set(HIGHS_INC "/usr/local/include/highs/")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "${HIGHS_INC}"
)

add_executable(afo_detector src/main.cpp src/jsoncpp.cpp src/segment.cpp)
target_link_libraries(afo_detector ${catkin_LIBRARIES} highs)
