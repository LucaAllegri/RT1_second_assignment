cmake_minimum_required(VERSION 3.8)
project(robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED) 
find_package(geometry_msgs REQUIRED) 
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED) 
find_package(robot_custom_msgs REQUIRED)

add_executable(ui src/ui_node.cpp)  
add_executable(distance src/distance_node.cpp) 
add_executable(robot_service src/robot_service_node.cpp) 

ament_target_dependencies(ui rclcpp geometry_msgs std_msgs sensor_msgs robot_custom_msgs)
ament_target_dependencies(distance rclcpp geometry_msgs std_msgs sensor_msgs robot_custom_msgs)
ament_target_dependencies(robot_service rclcpp robot_custom_msgs)


install(TARGETS 
  ui
  distance
  robot_service
  DESTINATION lib/${PROJECT_NAME}
)

install( 
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_package()
