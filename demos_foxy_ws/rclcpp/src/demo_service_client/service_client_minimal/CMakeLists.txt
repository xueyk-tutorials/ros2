cmake_minimum_required(VERSION 3.5)
project(service_client_minimal)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)

set(dependencies
  rclcpp
  example_interfaces
)
# minimal，来自ROS2官网教程
add_executable(minimal_service src/minimal_service.cpp)
ament_target_dependencies(minimal_service ${dependencies})
add_executable(minimal_client src/minimal_client.cpp)
ament_target_dependencies(minimal_client ${dependencies})

# 服务请求异步接收
add_executable(async_service src/async_service.cpp)
ament_target_dependencies(async_service ${dependencies})
add_executable(async_client src/async_client.cpp)
ament_target_dependencies(async_client ${dependencies})

install(TARGETS
  minimal_service
  minimal_client
  async_service
  async_client
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
