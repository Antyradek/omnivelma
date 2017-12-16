cmake_minimum_required(VERSION 2.8.3)
project(dziadzio)

find_package(catkin REQUIRED COMPONENTS
  omnivelma_msgs
  roscpp
)


catkin_package(
	CATKIN_DEPENDS 
	roscpp 
	omnivelma_msgs
)

find_package(roscpp REQUIRED)
find_package(omnivelma_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${omnivelma_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic --std=c++14")
add_executable(${PROJECT_NAME} src/dziadzio.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
