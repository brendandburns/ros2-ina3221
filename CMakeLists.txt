cmake_minimum_required(VERSION 3.8)
project(ros2-power-ina3221)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
)

add_executable(ros2_ina3221 src/ros2_ina3221.cpp)

add_library(
    ros2_ina3221_node
    SHARED
    src/node.cpp
)

ament_target_dependencies(ros2_ina3221_node rclcpp sensor_msgs)
ament_target_dependencies(ros2_ina3221 rclcpp sensor_msgs)

target_link_libraries(ros2_ina3221 ros2_ina3221_node)
target_link_libraries(ros2_ina3221_node i2c)

target_include_directories(ros2_ina3221 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(ros2_ina3221 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS ros2_ina3221
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS ros2_ina3221_node
  EXPORT ros2_ina3221_node 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch 
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
