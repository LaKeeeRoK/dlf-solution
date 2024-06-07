cmake_minimum_required(VERSION 3.0.2)
project(dlf-solution)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  hector_uav_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories (
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_move src/simple_move.cpp)
add_dependencies(simple_move ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_move ${OpenCV_LIBS} ${catkin_LIBRARIES})
