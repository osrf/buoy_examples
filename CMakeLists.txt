cmake_minimum_required(VERSION 3.8)
project(buoy_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(buoy_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(torque_controller src/torque_controller.cpp)
ament_target_dependencies(torque_controller rclcpp buoy_msgs)
target_include_directories(torque_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(torque_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS torque_controller
  DESTINATION lib/${PROJECT_NAME})


###########################################
# for buoy_tests
add_library(torque_policy_interface INTERFACE)
target_include_directories(torque_policy_interface INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS torque_policy_interface
  EXPORT export_torque_policy_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY include/JustInterp/
  DESTINATION include/JustInterp
)
###########################################

install(PROGRAMS
  scripts/torque_controller.py
  scripts/bias_damping.py
  DESTINATION lib/${PROJECT_NAME})

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
  ament_lint_auto_find_test_dependencies()
endif()

# for buoy_tests
ament_export_targets(export_torque_policy_interface)

ament_package()
