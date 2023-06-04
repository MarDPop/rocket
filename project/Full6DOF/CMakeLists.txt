cmake_minimum_required(VERSION 3.10)

# set the project name
project(Full6DOF VERSION 0.1)

include_directories(${PROJECT_SOURCE_DIR})

# add the executable
add_executable(Full6DOF test/main.cpp)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)