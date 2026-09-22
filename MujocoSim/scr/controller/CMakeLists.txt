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
find_package(rcl_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

set(CORE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../Core)
if(NOT EXISTS ${CORE_SOURCE_DIR}/src/controller.cpp)
  message(FATAL_ERROR "Core controller sources were not found at ${CORE_SOURCE_DIR}")
endif()

# Use the copy vendored in this workspace instead of an installed autodiff
# package, which may leave a stale or incomplete CMake package behind.
if(NOT TARGET autodiff::autodiff)
  add_subdirectory(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../third_party/autodiff
    ${CMAKE_CURRENT_BINARY_DIR}/autodiff
    EXCLUDE_FROM_ALL
  )
endif()

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
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

add_library(lqr_controller SHARED
  src/controller/controller.cpp
  src/controller/mujoco_sim.cpp
  src/controller/vir_imu.cpp
  src/controller/vir_motor.cpp
  ${CORE_SOURCE_DIR}/src/controllerbase.cpp
  ${CORE_SOURCE_DIR}/src/controller.cpp
  ${CORE_SOURCE_DIR}/src/lqr_gain_scheduler.cpp
  ${CORE_SOURCE_DIR}/src/leg_calc.cpp
)

target_include_directories(lqr_controller PRIVATE
  ${CORE_SOURCE_DIR}/inc
)

set(controller_common_dependencies
  rclcpp
  rcl_interfaces
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  urdf
)

target_include_directories(lqr_controller PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(lqr_controller Eigen3::Eigen autodiff::autodiff)

ament_target_dependencies(
  lqr_controller
  ${controller_common_dependencies}
)

install(TARGETS lqr_controller
  LIBRARY DESTINATION lib
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include)

install(DIRECTORY ${CORE_SOURCE_DIR}/inc/
DESTINATION include)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
