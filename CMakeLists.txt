cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(as_manager)
set(NODE_NAME ${PROJECT_NAME}_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(mmr_kria_base REQUIRED)
find_package(mmr_edf REQUIRED)

include_directories(include)

file(GLOB_RECURSE SOURCES include/*.cpp)
add_executable(
  ${NODE_NAME}
  src/${NODE_NAME}.cpp
  src/${PROJECT_NAME}.cpp
  ${SOURCES}
)

ament_target_dependencies(${NODE_NAME}
  rclcpp
  std_msgs
  can_msgs
  mmr_kria_base
  mmr_edf
)

install(
  TARGETS ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if (APPLE)
  set(CMAKE_CXX_COMPILER clang)
  add_link_options(-lstdc++)
endif()

# add_compile_definitions(TEST)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if (BUILD_TESTING)
endif()

ament_package()
