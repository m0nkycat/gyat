cmake_minimum_required(VERSION 3.8)
project(tb4_cpp_prac6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)

set(executable_name iar_amcl)


add_executable(${executable_name} src/iar_amcl.cpp src/iar_amcl_node.cpp src/iar_pf_kdtree)

target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${executable_name} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

set(dependencies
  "rclcpp"
  "rclcpp_lifecycle"
  "sensor_msgs"
  "tf2_ros"
  "tf2"
  "geometry_msgs"
  "nav2_amcl"
  "message_filters"
  "tf2_geometry_msgs"
  "nav_msgs"
  "std_srvs"
  "nav2_util"
  "nav2_msgs"
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)



install(TARGETS ${executable_name}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  maps
  DESTINATION share/${PROJECT_NAME}/
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
