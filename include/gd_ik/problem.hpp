#pragma once

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

#include <chrono>
#include <memory>
#include <vector>

#include "gd_ik/frame.hpp"
#include "gd_ik/goal.hpp"
#include "gd_ik/robot.hpp"

namespace gd_ik {

struct Problem {
  std::vector<ssize_t> link_tip_indices;
  std::vector<int> joint_usage;
  std::vector<double> minimal_displacement_factors;
  std::vector<double> joint_transmission_goal_temp;
  std::vector<double> joint_transmission_goal_temp2;

  std::chrono::time_point<std::chrono::system_clock,
                          std::chrono::duration<double>>
      timeout;
  std::vector<double> initial_guess;
  std::vector<size_t> active_variable_indices;
  std::vector<size_t> tip_link_indices;
  std::vector<Goal> goals;
  std::vector<Goal> secondary_goals;

  static Problem from(
      std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
      moveit::core::JointModelGroup const* jmg, std::vector<Goal> const& goals,
      Robot const& robot);
};

size_t add_tip_link(Problem& self, moveit::core::LinkModel const& link_model);
size_t add_active_variable(
    Problem& self,
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    std::string const& name);

}  // namespace gd_ik
