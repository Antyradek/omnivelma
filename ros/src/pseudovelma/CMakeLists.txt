cmake_minimum_required(VERSION 2.8.3)
project(pseudovelma)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  omnivelma
)

catkin_package(
	CATKIN_DEPENDS 
	std_msgs
	gazebo_ros
	roscpp 
	omnivelma
)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omnivelma REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS   "-Wall")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/pseudovelma.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})
