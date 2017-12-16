cmake_minimum_required(VERSION 2.8.3)
project(lalkarz)

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
include_directories(${PROJECT_SOURCE_DIR}/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)
add_executable(${PROJECT_NAME} 
	src/lalkarz.cpp
	src/bin_twist_state.cpp
	src/cont_vels_state.cpp
	src/steps_twist_state.cpp
	src/bin_vels_state.cpp
	src/gamepad_twist_state.cpp
	src/mouse_twist_state.cpp
	src/steps_vels_state.cpp
	src/cont_twist_state.cpp
	src/gamepad_vels_state.cpp
	src/state.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system pthread)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
