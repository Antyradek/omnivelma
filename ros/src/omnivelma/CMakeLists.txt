cmake_minimum_required(VERSION 2.8.3)
project(omnivelma)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  geometry_msgs
  omnivelma_msgs
  tf2_ros
)

catkin_package(
	CATKIN_DEPENDS 
	geometry_msgs
	gazebo_ros
	roscpp 
	omnivelma_msgs
	tf2_ros
)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(omnivelma_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_FLAGS   "-Wall -Wpedantic --std=c++14")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/omnivelma.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

add_library(monokl_chain src/monokl_chain.cpp)
add_dependencies(monokl_chain ${catkin_EXPORTED_TARGETS})
target_link_libraries(monokl_chain ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

