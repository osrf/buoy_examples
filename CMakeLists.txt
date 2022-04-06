cmake_minimum_required(VERSION 3.8)
project(pbcontrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(pb_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(torque_controller src/torque_controller.cpp)
ament_target_dependencies(torque_controller rclcpp pb_msgs)
target_include_directories(torque_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(torque_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS torque_controller
  DESTINATION lib/${PROJECT_NAME})

#add_executable(sandbox src/sandbox.cpp)
#ament_target_dependencies(sandbox rclcpp std_msgs)
#target_include_directories(torque_controller PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(sandbox PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

#install(TARGETS sandbox
#  DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/gain_scheduler_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
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
