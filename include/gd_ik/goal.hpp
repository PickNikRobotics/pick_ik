#pragma once

#include <moveit/robot_model/joint_model_group.h>

#include <functional>
#include <string>
#include <vector>

#include "gd_ik/frame.hpp"
#include "gd_ik/robot.hpp"

namespace gd_ik {

struct GoalContext {
  std::vector<double> active_variable_positions;
  std::vector<Frame> tip_link_frames;
  std::vector<ssize_t> goal_variable_indices;
  std::vector<size_t> goal_link_indices;
  bool goal_secondary;
  std::vector<std::string> goal_link_names;
  std::vector<std::string> goal_variable_names;
  double goal_weight;
  moveit::core::JointModelGroup const* joint_model_group;
  std::vector<size_t> problem_active_variables;
  std::vector<size_t> problem_tip_link_indices;
  std::vector<double> initial_guess;
  std::vector<double> velocity_weights;
  Robot robot;
  std::vector<double> temp_vector;
};

// Goal Function type
using GoalEvalFn =
    std::function<double(std::vector<Frame> const& tip_frames,
                         std::vector<double> const& active_positions)>;

struct Goal {
  GoalEvalFn eval;
  double weight;
  size_t tip_index;
  Frame frame;
  std::vector<std::string> link_names;
};

}  // namespace gd_ik
