cmake_minimum_required(VERSION 3.5)
project(merlin2_bt_action)

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
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(merlin2_action REQUIRED)

# C++
include_directories(include)
include_directories(src)

set(LIB ${CMAKE_PROJECT_NAME}_lib)
set(SOURCES
  src/merlin2_bt_action.cpp
)
set(DEPENDENCIES
  rclcpp
  behaviortree_cpp_v3
  merlin2_action
)

add_library(${LIB} STATIC ${SOURCES})
ament_target_dependencies(${LIB} ${DEPENDENCIES} Threads)
install(TARGETS ${LIB}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include/
)
ament_export_include_directories(include)
ament_export_libraries(${LIB})
ament_export_dependencies(${DEPENDENCIES})

# plugins
find_package(waypoint_navigation_interfaces REQUIRED)
set(PLUGINS_DEPENDENCIES
  rclcpp
  simple_node
  behaviortree_cpp_v3
  waypoint_navigation_interfaces
  merlin2_arch_interfaces
)

add_library(waypoint_navigation_node SHARED src/bt_nodes/waypoint_navigation.cpp)
ament_target_dependencies(waypoint_navigation_node ${PLUGINS_DEPENDENCIES})
list(APPEND PLUGIN_LIBS waypoint_navigation_node)
target_compile_definitions(waypoint_navigation_node PRIVATE BT_PLUGIN_EXPORT)

install(TARGETS ${PLUGIN_LIBS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PLUGIN_LIBS})
ament_export_dependencies(${PLUGINS_DEPENDENCIES})
ament_package()
