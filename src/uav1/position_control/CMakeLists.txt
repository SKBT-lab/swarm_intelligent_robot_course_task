cmake_minimum_required(VERSION 3.0.2)
project(position_control)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )

SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
# 查找依赖的Catkin包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
  minimumsnap
)


# Catkin包配置
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp quadrotor_msgs minimumsnap
)

# 包含目录
include_directories(
  SYSTEM  
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 创建可执行文件
add_executable(position_command_publisher 
  src/uav1.cpp
)



# 链接库
target_link_libraries(position_command_publisher
  ${catkin_LIBRARIES}
)

add_executable(position_command_publisher2 
  src/uav2.cpp 
)

target_link_libraries(position_command_publisher2
  ${catkin_LIBRARIES}
)
