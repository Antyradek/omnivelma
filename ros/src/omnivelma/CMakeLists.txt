cmake_minimum_required(VERSION 2.8.3)
project(omnivelma)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
	FILES
	Vels.msg
	Encoders.msg
)

add_service_files(
	FILES
	SetFriction.srv
	SetInertia.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)


catkin_package(
	CATKIN_DEPENDS 
	message_runtime
	std_msgs
	geometry_msgs
	gazebo_ros
	roscpp 
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
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})
