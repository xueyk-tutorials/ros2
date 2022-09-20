cmake_minimum_required(VERSION 3.5)
project(time_basic)

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

###
set(dependencies
  rclcpp
)
# rclcpp::Time
add_executable(minimal_time src/minimal_time.cpp)
ament_target_dependencies(minimal_time ${dependencies})
# rclcpp::Clock
add_executable(minimal_clock src/minimal_clock.cpp)
ament_target_dependencies(minimal_clock ${dependencies})
#
add_executable(clock_now src/clock_now.cpp)
ament_target_dependencies(clock_now ${dependencies})

add_executable(time_source1 src/time_source1.cpp)
ament_target_dependencies(time_source1 ${dependencies})

add_executable(time_header_publisher src/time_header_publisher.cpp)
ament_target_dependencies(time_header_publisher ${dependencies})
add_executable(time_header_subscription src/time_header_subscription.cpp)
ament_target_dependencies(time_header_subscription ${dependencies})
#
install(TARGETS
  minimal_time
  minimal_clock
  time_source1
  clock_now
  time_header_publisher
  time_header_subscription
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
