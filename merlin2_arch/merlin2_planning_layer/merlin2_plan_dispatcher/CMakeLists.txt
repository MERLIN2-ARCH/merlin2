cmake_minimum_required(VERSION 3.5)
project(merlin2_plan_dispatcher)

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


# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  ${PROJECT_NAME}/merlin2_plan_dispatcher_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plan_dispatcher_node
)

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
