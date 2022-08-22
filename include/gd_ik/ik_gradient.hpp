#pragma once

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

#include <memory>
#include <vector>

#include "gd_ik/frame.hpp"
#include "gd_ik/goal.hpp"
#include "gd_ik/robot.hpp"

namespace gd_ik {

struct Problem {
  bool ros_params_initrd;
  std::vector<int> joint_usage;
  std::vector<ssize_t> link_tip_indices;
  std::vector<double> minimal_displacement_factors;
  std::vector<double> joint_transmission_goal_temp;
  std::vector<double> joint_transmission_goal_temp2;
};

struct IkGradientDecent {
  std::shared_ptr<moveit::core::RobotModel const> robot_model;
  moveit::core::JointModelGroup const* jmg;

  Robot robot;
  Problem problem;

  std::vector<Frame> null_tip_frames;

  std::atomic<bool> canceled = false;

  bool reset;
  std::vector<size_t> active_variables;
  std::vector<double> solution;
  std::vector<double> gradient;
  std::vector<double> best_solution;
  std::vector<double> temp;
};

// void step(IkGradientDecent& self);

auto active_variable_positions(
    std::vector<size_t> const& active_variable_indices,
    std::vector<double> const& variables) -> std::vector<double>;

auto fitness(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
             std::vector<size_t> const& tip_link_indices,
             std::vector<size_t> const& active_variable_indices,
             std::vector<Goal> const& goals,
             std::vector<double> const& variables) -> double;

}  // namespace gd_ik
