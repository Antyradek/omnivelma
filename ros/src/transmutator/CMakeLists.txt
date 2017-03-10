cmake_minimum_required(VERSION 2.8.3)
project(transmutator)

find_package(catkin REQUIRED COMPONENTS
  omnivelma
  roscpp
  std_msgs
  geometry_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	std_msgs
	geometry_msgs
	roscpp 
	omnivelma
)

find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(omnivelma REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall")
add_executable(${PROJECT_NAME} src/transmutator.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
