cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER clang)

project(MmrAsManager)

file(GLOB_RECURSE SOURCES LIST_DIRECTORIES true lib/*.cpp)

add_executable(as_manager ${SOURCES})

include_directories(lib/)


