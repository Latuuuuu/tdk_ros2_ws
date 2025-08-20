cmake_minimum_required(VERSION 3.8)
project(main-pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)            
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)               
find_package(visualization_msgs REQUIRED) 
find_package(interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(SpeedPublisher src/speed_publisher.cpp)
ament_target_dependencies(SpeedPublisher rclcpp nav_msgs std_msgs geometry_msgs rcl_interfaces visualization_msgs interfaces)
target_include_directories(SpeedPublisher PRIVATE include)

add_executable(SimLower src/sim_lower.cpp)
target_include_directories(SimLower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(SimLower rclcpp geometry_msgs nav_msgs)

install(TARGETS
  SimLower
  SpeedPublisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY 
    launch
    rviz 
    DESTINATION share/${PROJECT_NAME}
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
