cmake_minimum_required(VERSION 3.5)
project(simple_line_follower)

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
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

# Create a library for line_follower_node
add_library(line_follower_library SHARED
  src/line_follower.cpp
)
ament_target_dependencies(line_follower_library 
  ${dependencies}
)

add_library(naive_line_follower_library SHARED
  src/naive_line_follower.cpp
)
ament_target_dependencies(naive_line_follower_library 
  ${dependencies}
)

add_executable(naive_line_follower_node src/naive_line_follower.cpp)
target_link_libraries(naive_line_follower_node naive_line_follower_library)

add_executable(line_follower_node src/line_follower.cpp)
target_link_libraries(line_follower_node line_follower_library)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)

ament_target_dependencies(naive_line_follower_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

ament_target_dependencies(line_follower_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

ament_target_dependencies(tf_broadcaster
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

target_link_libraries(line_follower_node ${rclcpp_LIBRARIES})

target_include_directories(line_follower_library PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
)

install(TARGETS
  naive_line_follower_node
  line_follower_node
  tf_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/
  DESTINATION include
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_subdirectory(test)  
endif()

ament_package()
