cmake_minimum_required(VERSION 3.22)
project(gd_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)

find_package(backward_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(range-v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)

generate_parameter_library(
  gd_ik_parameters
  src/gd_ik_parameters.yaml
)
add_library(gd_ik_plugin SHARED
  src/fk_moveit.cpp
  src/forward_kinematics.cpp
  src/frame.cpp
  src/gd_ik_plugin.cpp
  src/goal.cpp
  src/ik_gradient.cpp
  src/math.cpp
  src/robot.cpp
)
target_compile_features(gd_ik_plugin PUBLIC c_std_99 cxx_std_17)
target_include_directories(gd_ik_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(gd_ik_plugin
  PUBLIC
    Eigen3::Eigen
    moveit_core::moveit_kinematics_base
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    pluginlib::pluginlib
    rclcpp::rclcpp
    tf2::tf2
    tf2_eigen::tf2_eigen
    tf2_geometry_msgs::tf2_geometry_msgs
    tf2_kdl::tf2_kdl
  PRIVATE
    fmt::fmt
    gd_ik_parameters
    range-v3::range-v3
)

pluginlib_export_plugin_description_file(
  moveit_core
  gd_ik_kinematics_description.xml
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(gd_ik_plugin PRIVATE "GD_IK_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS gd_ik_plugin gd_ik_parameters
  EXPORT export_gd_ik
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(gd_ik_plugin)
ament_export_targets(export_gd_ik)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

ament_package()
