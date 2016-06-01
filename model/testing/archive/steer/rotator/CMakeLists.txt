cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(rotator SHARED rotator.cpp)
target_link_libraries(rotator ${GAZEBO_libraries}) 

# Build the stand-alone test program
add_executable(steer steer.cpp)

if (${gazebo_VERSION_MAJOR} LESS 6)
	# These two
	include(FindBoost)
	find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)
	target_link_libraries(steer ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
	else()
		target_link_libraries(steer ${GAZEBO_LIBRARIES})
		endif()
