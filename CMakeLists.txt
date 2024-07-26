cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER clang)

project(MmrAsManager)

include_directories(lib/)
add_link_options(
  -lstdc++
)

file(GLOB_RECURSE SOURCES lib/*.cpp)
add_executable(as_manager main.cpp ${SOURCES})
