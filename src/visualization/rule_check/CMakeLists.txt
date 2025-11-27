cmake_minimum_required(VERSION 3.0.2)
project(rule_check)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rule_check
  CATKIN_DEPENDS nav_msgs pcl_conversions pcl_ros roscpp sensor_msgs std_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(task2_rule src/task2.cpp)
target_link_libraries(task2_rule
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(task1_rule src/task1.cpp)
target_link_libraries(task1_rule
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
