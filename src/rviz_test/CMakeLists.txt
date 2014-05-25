cmake_minimum_required(VERSION 2.8.3)
project(rviz_test)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(RvizL2T src/RvizL2T.cpp)
target_link_libraries(RvizL2T ${catkin_LIBRARIES})
add_dependencies(RvizL2T ${catkin_EXPORTED_TARGETS})

add_executable(RvizL2Ttest src/RvizL2Ttest.cpp)
target_link_libraries(RvizL2Ttest ${catkin_LIBRARIES})
add_dependencies(RvizL2Ttest ${catkin_EXPORTED_TARGETS})
