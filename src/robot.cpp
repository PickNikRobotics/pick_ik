#include "gd_ik/robot.hpp"

#include <cfloat>
#include <cmath>

#include "gd_ik/math.hpp"

namespace gd_ik {

auto Robot::from(std::shared_ptr<moveit::core::RobotModel const> const& model)
    -> Robot {
  auto robot = Robot{};

  for (auto const& name : model->getVariableNames()) {
    auto const& bounds = model->getVariableBounds(name);

    auto var = Variable{};

    bool bounded = bounds.position_bounded_;

    auto* joint_model = model->getJointOfVariable(robot.variables.size());
    if (dynamic_cast<moveit::core::RevoluteJointModel const*>(joint_model)) {
      if (bounds.max_position_ - bounds.min_position_ >= 2 * M_PI * 0.9999) {
        bounded = false;
      }
    }

    var.min = bounds.min_position_;
    var.max = bounds.max_position_;

    var.clip_min = bounded ? var.min : -DBL_MAX;
    var.clip_max = bounded ? var.max : +DBL_MAX;

    var.span = var.max - var.min;

    if (!(var.span >= 0 && var.span < FLT_MAX)) var.span = 1;

    var.max_velocity = bounds.max_velocity_;
    var.max_velocity_rcp =
        var.max_velocity > 0.0 ? 1.0 / var.max_velocity : 0.0;

    robot.variables.push_back(var);
  }

  for (size_t ivar = 0; ivar < model->getVariableCount(); ++ivar) {
    robot.variable_joint_types.push_back(
        model->getJointOfVariable(ivar)->getType());
  }

  return robot;
}

auto get_span(Robot const& self, size_t i) -> double {
  return self.variables.at(i).span;
}

auto get_clip_min(Robot const& self, size_t i) -> double {
  return self.variables.at(i).clip_min;
}

auto get_clip_max(Robot const& self, size_t i) -> double {
  return self.variables.at(i).clip_max;
}

auto get_min(Robot const& self, size_t i) -> double {
  return self.variables.at(i).min;
}

auto get_max(Robot const& self, size_t i) -> double {
  return self.variables.at(i).max;
}

auto is_revolute(Robot const& self, size_t variable_index) -> bool {
  return self.variable_joint_types.at(variable_index) ==
         moveit::core::JointModel::REVOLUTE;
}

auto is_prismatic(Robot const& self, size_t variable_index) -> bool {
  return self.variable_joint_types.at(variable_index) ==
         moveit::core::JointModel::PRISMATIC;
}

auto get_max_velocity(Robot const& self, size_t i) -> double {
  return self.variables.at(i).max_velocity;
}

auto get_max_velocity_rcp(Robot const& self, size_t i) -> double {
  return self.variables.at(i).max_velocity_rcp;
}

auto clip(Robot const& self, double p, size_t i) -> double {
  auto const& info = self.variables.at(i);
  return clamp2(p, info.clip_min, info.clip_max);
}

}  // namespace gd_ik
