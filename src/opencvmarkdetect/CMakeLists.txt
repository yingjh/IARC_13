cmake_minimum_required(VERSION 2.8.3)
project(opencvmarkdetect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

add_message_files(FILES Xyz.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${OpenCV_INCLUDE_DIR})

link_directories(${OpenCV_LIBRARY_DIR})

link_libraries(${OpenCV_LIBS})

SET(SRC
./src/main.cpp
./src/Marker.cpp
./src/MarkerDetector.cpp
)


add_executable(${PROJECT_NAME}  ${SRC})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} opencvmarkdetect_generate_messages_cpp)


add_executable(Marklistener  src/listener.cpp)
target_link_libraries(Marklistener ${catkin_LIBRARIES})
add_dependencies(Marklistener opencvmarkdetect_generate_messages_cpp)

add_executable(points_and_lines src/points_and_lines.cpp)
target_link_libraries(points_and_lines ${catkin_LIBRARIES})

add_executable(basic_shapes src/basic_shapes.cpp)
target_link_libraries(basic_shapes ${catkin_LIBRARIES})

add_executable(pointtest src/pointtest.cpp)
target_link_libraries(pointtest ${catkin_LIBRARIES})

add_executable(Rvizlistener  src/Rvizlistener.cpp)
target_link_libraries(Rvizlistener  ${catkin_LIBRARIES})
add_dependencies(Rvizlistener  opencvmarkdetect_generate_messages_cpp)

