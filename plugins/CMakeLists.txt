cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Ustaw ostrzeżenia
set(CMAKE_C_FLAGS   "-Wall")

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(omnivelma SHARED omnivelma.cpp)
target_link_libraries(omnivelma ${GAZEBO_libraries})

