cmake_minimum_required(VERSION 2.8.3)
project(odszumiacz)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
)


catkin_package(
	CATKIN_DEPENDS 
	roscpp 
	sensor_msgs
)

find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic --std=c++14")
add_executable(${PROJECT_NAME} src/odszumiacz.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
