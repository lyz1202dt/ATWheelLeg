cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)
find_package(autodiff REQUIRED)

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

pluginlib_export_plugin_description_file(controller_interface plugin.xml)
pluginlib_export_plugin_description_file(controller_interface mpc_controller_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

add_subdirectory(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/TinyMPC/src/tinympc
  ${CMAKE_CURRENT_BINARY_DIR}/tinympc
)

add_library(lqr_controller SHARED
  src/controller/controller.cpp
  src/controller/mujoco_sim.cpp
  src/controller/real_hardware.cpp
  src/tools/leg_calc.cpp
)

add_library(mpc_controller SHARED
  src/controller/mpc_controller.cpp
  src/tools/leg_calc.cpp
)

set(controller_common_dependencies
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  robot_interfaces
  urdf
)

foreach(controller_target lqr_controller mpc_controller)
  target_include_directories(${controller_target} PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(${controller_target} Eigen3::Eigen autodiff::autodiff)

  ament_target_dependencies(
    ${controller_target}
    ${controller_common_dependencies}
  )
endforeach()

target_link_libraries(mpc_controller tinympcstatic)

install(TARGETS lqr_controller mpc_controller
  LIBRARY DESTINATION lib
)

install(
  FILES plugin.xml mpc_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
