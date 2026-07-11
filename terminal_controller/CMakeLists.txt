cmake_minimum_required(VERSION 3.8)
project(robonix_nav2_terminal)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/persistent_goal_checker.cpp
  src/persistent_rotate_to_goal_critic.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  angles dwb_core dwb_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav2_core
  nav2_costmap_2d nav2_util pluginlib rclcpp rclcpp_lifecycle tf2)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)
pluginlib_export_plugin_description_file(dwb_core plugins.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_persistent_goal_checker test/test_persistent_goal_checker.cpp)
  target_link_libraries(test_persistent_goal_checker ${PROJECT_NAME})
  ament_target_dependencies(test_persistent_goal_checker
    dwb_core geometry_msgs nav2_core pluginlib rclcpp rclcpp_lifecycle tf2)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nav2_core dwb_core)
ament_package()
