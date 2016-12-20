cmake_minimum_required(VERSION 2.8.3)
project(omnivelma)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  message_generation
)

add_message_files(
	FILES
	Vels.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
)


catkin_package(
	DEPENDS 
	roscpp 
	gazebo_ros 
	message_runtime
)

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})

set(CMAKE_C_FLAGS   "-Wall")
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} src/omnivelma.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${roscpp_LIBRARIES})

