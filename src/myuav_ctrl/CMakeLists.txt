cmake_minimum_required(VERSION 2.8.3)
project(myuav_ctrl)

find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs cv_bridge image_transport)
find_package(OpenCV REQUIRED)

## code to find serial communication library
FIND_PATH(SERIAL_INCLUDE_DIR SerialStream.h
   /usr/include
   /usr/local/include
)

FIND_LIBRARY(SERIAL_LIBRARY NAMES  serial libserial
   PATHS
   /usr/lib
   /usr/local/lib
)

if (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
   set(SERIAL_FOUND TRUE)
   set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
else (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
   set(SERIAL_FOUND FALSE)
endif (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)

if (SERIAL_FOUND)
      message(STATUS "Found LibSerial: ${SERIAL_LIBRARIES}")
else (SERIAL_FOUND)
	   message(STATUS "don't find LibSerial")
endif (SERIAL_FOUND)
MARK_AS_ADVANCED(SERIAL_INCLUDE_DIR SERIAL_LIBRARIES SERIAL_LIBRARY)


####################################
catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(${catkin_INCLUDE_DIRS})

##################################

add_executable(myserial_test src/serialUAV.cpp)
target_link_libraries(myserial_test
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(myserial_test ${catkin_EXPORTED_TARGETS})

add_executable(faceuav_test src/faceUAV.cpp)
target_link_libraries(faceuav_test
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
  ${OpenCV_LIBS}
)
add_dependencies(faceuav_test ${catkin_EXPORTED_TARGETS})



