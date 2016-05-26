cmake_minimum_required(VERSION 2.8.3)
project(rosbot_path_finder)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  tf
  cv_bridge
  sensor_msgs
  rospy
  std_msgs
  image_transport
)
find_package(OpenCV 2)
find_package(Boost REQUIRED COMPONENTS system)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
catkin_package()
add_executable(rosbot_path_finder_node src/rosbot_path_finder_node.cpp)
target_link_libraries(rosbot_path_finder_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES})
