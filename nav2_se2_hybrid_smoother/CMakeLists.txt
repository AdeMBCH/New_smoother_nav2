cmake_minimum_required(VERSION 3.10)
project(nav2_se2_hybrid_smoother)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/se2_hybrid_smoother.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_util
  pluginlib
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config bt launch scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
