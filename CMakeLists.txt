cmake_minimum_required(VERSION 3.0.2)
project(rosclips64)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost COMPONENTS thread REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

find_library(
	CLIPS_LIB
	NAME	clips-6.24
			clips
	PATHS 	${PROJECT_SOURCE_DIR}/lib
	NO_DEFAULT_PATH
)

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

message(STATUS "CSD: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "PSD: ${PROJECT_SOURCE_DIR}")
message(STATUS "CLP: ${CLIPS_LIB}")


## Build clips_node
include_directories(include
					${catkin_INCLUDE_DIRS}
					${PROJECT_SOURCE_DIR}/include
)

set( CMAKE_VERBOSE_MAKEFILE on )

add_library(clips SHARED IMPORTED)
set_target_properties(clips PROPERTIES
	IMPORTED_LOCATION "${CLIPS_LIB}"
	INCLUDE_DIRECTORIES "${PROJECT_SOURCE_DIR}/include/clips"
)

add_executable(rosclips64
	src/node.cpp
	src/CLIPSCore.cpp
)

target_link_libraries(rosclips64
								clips
								${catkin_LIBRARIES}
# 								${Boost_LIBRARIES}
)

# add_dependencies(clips_node clips)

add_executable(console src/console.cpp)
target_link_libraries(console ${catkin_LIBRARIES})
