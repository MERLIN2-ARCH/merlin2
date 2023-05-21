cmake_minimum_required(VERSION 3.8)
project(merlin2_demo)

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
find_package(kant_dto REQUIRED)
find_package(merlin2_bt_action REQUIRED)

# C++
include_directories(include)
set(DEPENDENCIES
  rclcpp
  kant_dto
  merlin2_bt_action
)

add_executable(merlin2_navigation_bt_action
  src/merlin2_navigation_bt_action.cpp
)
ament_target_dependencies(merlin2_navigation_bt_action ${DEPENDENCIES})
install(TARGETS
  merlin2_navigation_bt_action
  DESTINATION lib/${PROJECT_NAME}
)

# Python
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  merlin2_demo/merlin2_demo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_demo_node
)

install(PROGRAMS
  merlin2_demo/merlin2_demo2_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_demo2_node
)

install(PROGRAMS
  merlin2_demo/merlin2_demo3_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_demo3_node
)

install(PROGRAMS
  merlin2_demo/merlin2_hi_navigation_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_hi_navigation_action
)

install(PROGRAMS
  merlin2_demo/merlin2_hi_navigation_fsm_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_hi_navigation_fsm_action
)

install(PROGRAMS
  merlin2_demo/merlin2_navigation_fsm_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_navigation_fsm_action
)

install(PROGRAMS
  merlin2_demo/merlin2_check_wp_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_check_wp_action
)

install(PROGRAMS
  merlin2_demo/merlin2_check_wp_fsm_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_check_wp_fsm_action
)

install(PROGRAMS
  merlin2_demo/merlin2_check_door_fsm_action.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_check_door_fsm_action
)

install(PROGRAMS
  merlin2_demo/merlin2_listen_audio_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME merlin2_listen_audio_node
)

#
install(DIRECTORY
  launch params bt_xml
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()