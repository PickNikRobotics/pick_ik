#pragma once

#include <gd_ik/frame.hpp>
#include <gd_ik/robot.hpp>

#include <functional>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <string>
#include <vector>

namespace gd_ik {

// Test if a frame satisfies a goal
using FrameTestFn = std::function<bool(Frame const& tip_frame)>;

auto make_frame_test(Frame goal_frame, double position_threashold,
                     double rotation_threashold, double twist_threshold)
    -> FrameTestFn;

// Goal Function type
using CostFn =
    std::function<double(std::vector<double> const& active_positions)>;

struct Goal {
  CostFn eval;
  double weight;
  std::vector<std::string> link_names;
};

auto fitness(std::vector<Goal> const& goals,
             std::vector<Frame> const& tip_frames,
             std::vector<double> const& active_positions) -> double;

auto make_center_joints_cost_fn(
    Robot robot, std::vector<size_t> active_variable_indexes,
    std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_avoid_joint_limits_cost_fn(
    Robot robot, std::vector<size_t> active_variable_indexes,
    std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_minimal_displacement_cost_fn(
    std::vector<double> initial_guess,
    std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_ik_cost_fn(geometry_msgs::msg::Pose pose,
                     kinematics::KinematicsBase::IKCostFn cost_fn,
                     std::shared_ptr<moveit::core::RobotModel> robot_model,
                     moveit::core::JointModelGroup* jmg,
                     std::vector<double> initial_guess) -> CostFn;

}  // namespace gd_ik
