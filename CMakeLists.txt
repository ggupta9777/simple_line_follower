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

add_executable(naive_line_follower_node src/naive_line_follower.cpp)
add_executable(line_follower_node src/line_follower.cpp)

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


install(TARGETS
  naive_line_follower_node
  line_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

# If you have other directories to install (like launch or config), add them here
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Include Google Test in CMake's testing framework
  find_package(ament_cmake_gtest REQUIRED)
  
  # Add a test executable
  ament_add_gtest(test_line_follower test/test_line_follower.cpp)

  # Link your test executable with the library you want to test
  target_link_libraries(test_line_follower
    line_follower_node        # If you are testing the executable directly
  )

  # Include directories for the test targets
  target_include_directories(test_line_follower PUBLIC
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    # any other include dirs you need
  )
endif()

ament_package()
