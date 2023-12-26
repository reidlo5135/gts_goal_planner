cmake_minimum_required(VERSION 3.8)
project(gts_goal_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  include/${PROJECT_NAME}
)

find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(actionlib_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(gts_msgs REQUIRED)

set(source_sets
  src/gts_converter.c
  src/gts_goal_planner.c
)

set(executable_name gts_goal_planner)

add_executable(${executable_name}
  ${source_sets}
)

set(dependencies
  rcl
  rclc
  gts_msgs
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name}
  m
)

install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
