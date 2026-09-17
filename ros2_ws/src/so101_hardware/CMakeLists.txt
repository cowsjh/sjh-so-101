cmake_minimum_required(VERSION 3.8)
project(so101_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

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

# Feetech SCServo SDK (STS3215): built from repo-root opensrc/, static PIC lib linked into this plugin.
# ponytail: relative path to gitignored opensrc/; local learning repo only. Vendor into package if repo must move.
add_subdirectory(../../../opensrc/SCServo_Linux ${CMAKE_CURRENT_BINARY_DIR}/scservo)

add_library(so101_hardware SHARED src/so101_system.cpp)
target_include_directories(so101_hardware PUBLIC include)
ament_target_dependencies(so101_hardware hardware_interface pluginlib rclcpp)
target_link_libraries(so101_hardware SCServo)  # PUBLIC include dir comes transitively -> #include "scservo/SMS_STS.h"

pluginlib_export_plugin_description_file(hardware_interface so101_hardware.xml)

install(
  TARGETS
  so101_hardware
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
install (DIRECTORY include/ DESTINATION include)

ament_export_libraries(so101_hardware)
ament_package()
