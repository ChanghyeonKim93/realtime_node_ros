cmake_minimum_required(VERSION 2.8.3)
project(realtime_node)

find_package(catkin REQUIRED cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  sensor_msgs
  std_msgs
  message_filters
)

catkin_package(
		DEPENDS Eigen
)

set(includeDirs ${includeDirs} ${CMAKE_CURRENT_BINARY_DIR})
set(includeDirs ${includeDirs} include)

set(SRC ${SRC} src/main.cpp)
set(SRC ${SRC} src/MyAlgorithm.cpp)
set(SRC ${SRC} include/MyAlgorithm.h)

include_directories( ${includeDirs} )
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories( ${Eigen_INCLUDE_DIRS} )

set(CMAKE_CXX_FLAGS "-msse3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(DCMAKE_build_type debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -O2 -m64 -Ofast -flto -msse2 -march=native -mfpmath=sse")

add_executable (realtime_node ${SRC} )
target_link_libraries (realtime_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries (realtime_node ${Eigen_LIBRARIES} )
