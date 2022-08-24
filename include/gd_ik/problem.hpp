#pragma once

#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/robot.hpp>

#include <chrono>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <vector>

namespace gd_ik {

struct Problem {
  std::vector<size_t> active_variable_indexes;

  std::vector<double> minimal_displacement_factors;
  std::vector<double> joint_transmission_goal_temp;
  std::vector<double> joint_transmission_goal_temp2;

  std::chrono::time_point<std::chrono::system_clock,
                          std::chrono::duration<double>>
      timeout;

  std::vector<double> initial_guess;
  std::vector<size_t> tip_link_indexes;
  std::vector<Goal> goals;
  std::vector<Goal> secondary_goals;

  static auto from(
      std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
      moveit::core::JointModelGroup const* jmg, std::vector<Goal> const& goals,
      Robot const& robot) -> Problem;
};

// Calculates the active variable indexes
auto get_active_variable_indexes(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    moveit::core::JointModelGroup const* jmg,
    std::vector<size_t> const& tip_link_indexes) -> std::vector<size_t>;

auto add_active_variable(
    Problem& self,
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    std::string const& name) -> size_t;

}  // namespace gd_ik
