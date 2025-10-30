cmake_minimum_required(VERSION 3.0.2)
project(reeds_shepp_curves)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reeds_shepp_lib
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(reeds_shepp_lib
  src/reeds_shepp.cpp
  src/theta_star_planner.cpp
  src/utils.cpp
)

add_executable(demo src/demo.cpp)
target_link_libraries(demo
  reeds_shepp_lib
  ${catkin_LIBRARIES}
)
