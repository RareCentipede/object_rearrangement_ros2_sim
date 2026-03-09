cmake_minimum_required(VERSION 3.8)
project(mpnp_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  ament_index_cpp
  gz-plugin2
  gz-cmake3
  gz-sim8
  gz-sensors8
  rclcpp
  rclcpp_action
  ros_gz_bridge
  tf2_ros
  mpnp_interfaces
  geometry_msgs
  controller_manager_msgs
  angles
  rclpy
  sensor_msgs
  yaml-cpp
)

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Contact plugin
add_library(ContactPlugin SHARED src/contact_plugin.cpp)
target_include_directories(ContactPlugin PUBLIC include)

ament_target_dependencies(ContactPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Vacuum Tool Plugin
add_library(VacuumToolPlugin SHARED src/vacuum_tool_plugin.cpp)
target_include_directories(VacuumToolPlugin PUBLIC include)

ament_target_dependencies(VacuumToolPlugin
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(ContactPlugin VacuumToolPlugin)

install(
  TARGETS
    ContactPlugin
    VacuumToolPlugin
  DESTINATION
    lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
