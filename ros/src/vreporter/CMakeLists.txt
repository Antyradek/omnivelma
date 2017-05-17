cmake_minimum_required(VERSION 2.8.3)
project(vreporter)

find_package(catkin REQUIRED COMPONENTS
  omnivelma_msgs
  roscpp
)


catkin_package(
	CATKIN_DEPENDS 
	roscpp 
	omnivelma_msgs
)

if("$ENV{VREP_ROOT}" STREQUAL "")
	message(SEND_ERROR "Zmienna VREP_ROOT nie jest ustawiona, nie można skompilować pakietu.")
endif()

find_package(roscpp REQUIRED)
find_package(omnivelma_msgs REQUIRED)
include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${omnivelma_msgs_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "-Wall --std=c++14")
include_directories($ENV{VREP_ROOT}/programming/remoteApi)
include_directories($ENV{VREP_ROOT}/programming/include)
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)
add_definitions(-D__linux)
add_executable(${PROJECT_NAME} 
	src/vreporter.cpp
	$ENV{VREP_ROOT}/programming/remoteApi/extApi.c
	$ENV{VREP_ROOT}/programming/remoteApi/extApiPlatform.c
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} pthread)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
