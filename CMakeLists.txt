cmake_minimum_required(VERSION 3.8)
project(riptide_hardware_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(riptide_simulator REQUIRED)

find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)

# Set hardware_sim_sources
set(hardware_sim_sources
  src/imu_hardware_sim.cpp
  src/actuators_hardware_sim.cpp
  src/echosounder_hardware_sim.cpp
  src/pressure_hardware_sim.cpp
  src/battery_hardware_sim.cpp
)

# Compile riptide_hardware_sim
add_library(${PROJECT_NAME} SHARED ${hardware_sim_sources})
target_include_directories(${PROJECT_NAME} PRIVATE include ${riptide_simulator_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ignition-transport11 ignition-msgs8)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ignition-transport11
  ignition-msgs8
  riptide_simulator
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface riptide_hardware.xml)

# INSTALL
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)

  # ament_add_gmock(test_sparton_ahrs_m1_hwi test/test_sparton_ahrs_m1_hwi.cpp)
  # target_include_directories(test_sparton_ahrs_m1_hwi PRIVATE include)
  # target_link_libraries(test_sparton_ahrs_m1_hwi sparton_ahrs_m1_hardware_interface)
  # ament_target_dependencies(test_sparton_ahrs_m1_hwi
  #   pluginlib
  #   ros2_control_test_assets
  # )
endif()

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ignition-transport11
  ignition-msgs8
  riptide_simulator
)
ament_package()
