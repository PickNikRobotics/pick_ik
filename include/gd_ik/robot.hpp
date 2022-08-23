#pragma once

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/robot_model.h>

#include <vector>

namespace gd_ik {

struct Robot {
  struct Variable {
    double clip_min, clip_max;
    double span;
    double min;
    double max;
    double max_velocity, max_velocity_rcp;
  };
  std::vector<Variable> variables;
  std::vector<size_t> activeVariables;
  std::vector<moveit::core::JointModel::JointType> variable_joint_types;

  // Create new Robot from a RobotModel
  static auto from(std::shared_ptr<moveit::core::RobotModel const> const& model)
      -> Robot;
};

auto get_span(Robot const& self, size_t i) -> double;
auto get_clip_min(Robot const& self, size_t i) -> double;
auto get_clip_max(Robot const& self, size_t i) -> double;
auto get_min(Robot const& self, size_t i) -> double;
auto get_max(Robot const& self, size_t i) -> double;
auto is_revolute(Robot const& self, size_t variable_index) -> bool;
auto is_prismatic(Robot const& self, size_t variable_index) -> bool;
auto get_max_velocity(Robot const& self, size_t i) -> double;
auto get_max_velocity_rcp(Robot const& self, size_t i) -> double;
auto clip(Robot const& self, double p, size_t i) -> double;

auto get_link_indexes(
    std::shared_ptr<moveit::core::RobotModel const> const& model,
    std::vector<std::string> const& names) -> std::vector<size_t>;

}  // namespace gd_ik
