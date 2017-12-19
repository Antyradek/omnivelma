cmake_minimum_required(VERSION 2.8.3)
project(pseudovelma)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  omnivelma_msgs
  nav_msgs
)

catkin_package(
	CATKIN_DEPENDS 
	gazebo_ros
	roscpp 
	omnivelma_msgs
	geometry_msgs
	nav_msgs
)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(omnivelma_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++11")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/pseudovelma.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
