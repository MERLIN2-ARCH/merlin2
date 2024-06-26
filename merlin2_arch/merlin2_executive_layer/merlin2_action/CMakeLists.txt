cmake_minimum_required(VERSION 3.8)
project(merlin2_action)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(simple_node REQUIRED)
find_package(kant_dto REQUIRED)
find_package(kant_dao REQUIRED)
find_package(merlin2_msgs REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/merlin2_action.cpp
)
set(DEPENDENCIES
  rclcpp
  rclcpp_action
  simple_node
  kant_dto
  kant_dao
  merlin2_msgs
)
add_library(${LIB} STATIC ${SOURCES})

install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)

ament_target_dependencies(${LIB} ${DEPENDENCIES})
ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})

# Python
ament_python_install_package(${PROJECT_NAME})

# Tests
install(
  DIRECTORY tests/python/
  DESTINATION pytests/
)

install(
  PROGRAMS tests/python/run_pytests.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
