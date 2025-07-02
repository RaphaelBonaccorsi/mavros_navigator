cmake_minimum_required(VERSION 3.5)
project(mavros_navigator)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mavros_navigator_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclpy
    geometry_msgs
    mavros_msgs
    std_srvs
    mavros_navigator_interfaces
)

install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/mavros_navigator.py
  scripts/path_planner.py
  scripts/data_logger.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_dependencies(${dependencies})

ament_package()
