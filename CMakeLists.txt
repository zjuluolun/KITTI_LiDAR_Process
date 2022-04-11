cmake_minimum_required(VERSION 2.8)
project(sortscan)


find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ./src/)

add_executable(sortscan src/sortScan.cpp)

target_link_libraries(sortscan ${PCL_LIBRARIES})
