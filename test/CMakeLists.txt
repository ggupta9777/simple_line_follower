ament_add_gtest(test_line_follower 
    test_line_follower.cpp
)
ament_target_dependencies(test_line_follower
  ${dependencies}
)
target_link_libraries(test_line_follower
    line_follower_library        
)
target_include_directories(test_line_follower PUBLIC
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
)


ament_add_gtest(test_line_follower_good 
    test_line_follower_good.cpp
)
ament_target_dependencies(test_line_follower_good
  ${dependencies}
)
target_link_libraries(test_line_follower_good
    line_follower_library        
)

target_include_directories(test_line_follower_good PUBLIC
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
)


ament_add_gtest(test_line_follower_poor 
    test_line_follower_poor.cpp
)
ament_target_dependencies(test_line_follower_poor
  ${dependencies}
)
target_link_libraries(test_line_follower_poor
    naive_line_follower_library        
)

target_include_directories(test_line_follower_poor PUBLIC
    ${geometry_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
)




