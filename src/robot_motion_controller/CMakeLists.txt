cmake_minimum_required(VERSION 3.8)
project(robot_motion_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(person_follower src/person_follower.cpp)
target_include_directories(person_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(person_follower PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  person_follower
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
)

add_executable(wall_follower src/wall_follower.cpp)
target_include_directories(wall_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wall_follower PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  wall_follower
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
)


install(TARGETS person_follower
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS wall_follower
  DESTINATION lib/${PROJECT_NAME})
  
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
