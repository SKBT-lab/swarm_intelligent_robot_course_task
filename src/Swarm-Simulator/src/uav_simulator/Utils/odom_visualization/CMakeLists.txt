cmake_minimum_required(VERSION 2.8.3)
project(odom_visualization)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  visualization_msgs
  quadrotor_msgs
  tf
  pose_utils
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES irobot_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs
#  DEPENDS system_lib
)

find_package(Armadillo REQUIRED)
include_directories(${ARMADILLO_INCLUDE_DIRS})


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(odom_visualization src/odom_visualization.cpp)

target_link_libraries(odom_visualization
   ${catkin_LIBRARIES}
   ${ARMADILLO_LIBRARIES}
   pose_utils
)

add_executable(camera_tf_broadcaster src/camera_tf_broadcaster.cpp)
target_link_libraries(camera_tf_broadcaster ${catkin_LIBRARIES})
