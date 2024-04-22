cmake_minimum_required(VERSION 3.8)
project(systems_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_cascade_lifecycle REQUIRED)

find_package(attention_system REQUIRED)
find_package(perception_system REQUIRED)


set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_cascade_lifecycle

  attention_system
  perception_system
)

add_executable(systems_main src/systems_main.cpp)
ament_target_dependencies(systems_main ${dependencies})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  systems_main
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
