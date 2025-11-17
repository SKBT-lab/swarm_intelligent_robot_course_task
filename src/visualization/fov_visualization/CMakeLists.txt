cmake_minimum_required(VERSION 3.0.2)
project(fov_visualization)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找catkin和其他依赖包
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  pcl_conversions
  pcl_ros
  std_msgs
  tf2
  tf2_geometry_msgs
)

# 查找PCL
find_package(PCL REQUIRED)

# 查找Eigen
find_package(Eigen3 REQUIRED)

# Catkin特定配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES exploration_visualization
  CATKIN_DEPENDS 
    roscpp 
    visualization_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs 
    pcl_conversions 
    pcl_ros 
    std_msgs
  DEPENDS 
    PCL
    Eigen3
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

# 链接目录
link_directories(
  ${catkin_LIBRARIES}
  ${PCL_LIBRARY_DIRS}
)


add_executable(fov_visualizer src/fov_vis.cpp)
target_link_libraries(fov_visualizer ${catkin_LIBRARIES})

add_executable(fov_visualizer2 src/fov_vis2.cpp)
target_link_libraries(fov_visualizer2 ${catkin_LIBRARIES})

add_executable(region_visualizer src/region_vis.cpp)
target_link_libraries(region_visualizer ${catkin_LIBRARIES})

add_executable(region_visualizer2 src/region_vis2.cpp)
target_link_libraries(region_visualizer2 ${catkin_LIBRARIES})

install(TARGETS fov_visualizer
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
