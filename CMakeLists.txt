cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_COMPILER clang)

project(MmrAsManager)
add_executable(as_manager main.cpp)

include_directories(./lib/)
