cmake_minimum_required(VERSION 3.0.2)
project(target_publisher)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)
find_package(rqt_gui_cpp REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(publish_target src/publish_target.cpp)
target_link_libraries(publish_target
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
