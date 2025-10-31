cmake_minimum_required(VERSION 3.0.2)
project(afo_detector)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  highs REQUIRED
)

find_package(cmake_modules REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

catkin_package(
  DEPENDS Eigen
)
set(HIGHS_INC "/usr/local/include/highs/")
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  "${HIGHS_INC}"
)

add_executable(afo_detector src/main.cpp src/jsoncpp.cpp src/segment.cpp)
target_link_libraries(afo_detector ${catkin_LIBRARIES} ${Eigen_LIBRARIES} highs)
