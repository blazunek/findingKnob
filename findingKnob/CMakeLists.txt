cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(findKnob)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (findKnob findKnob.cpp)
target_link_libraries (findKnob ${PCL_LIBRARIES})

set_target_properties( findKnob PROPERTIES CXX_STANDARD 14)
