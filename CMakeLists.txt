cmake_minimum_required(VERSION 3.0.2)
project(sliding_map)

# Set target C++ standard and required compiler features
# set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

add_library(${PROJECT_NAME}
    sliding_map.cpp
)

target_include_directories (${PROJECT_NAME} PUBLIC 
    ${PROJECT_SOURCE_DIR} 
    ${EIGEN3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)