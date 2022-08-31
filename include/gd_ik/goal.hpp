#pragma once

#include <gd_ik/fk_moveit.hpp>
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

auto make_frame_tests(Frame goal_frame, double twist_threshold) -> FrameTestFn;

auto make_frame_tests(std::vector<Frame> goal_frames, double twist_threshold)
    -> std::vector<FrameTestFn>;

using PoseCostFn = std::function<double(std::vector<Frame> const& tip_frames)>;
auto make_pose_cost_fn(Frame goal, size_t goal_link_index, double rotation_scale) -> PoseCostFn;

auto make_pose_cost_functions(std::vector<Frame> goal_frames, double rotation_scale)
    -> std::vector<PoseCostFn>;

// Goal Function type
using CostFn = std::function<double(std::vector<double> const& active_positions)>;

struct Goal {
    CostFn eval;
    double weight;
};

auto make_center_joints_cost_fn(Robot robot,
                                std::vector<size_t> active_variable_indexes,
                                std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_avoid_joint_limits_cost_fn(Robot robot,
                                     std::vector<size_t> active_variable_indexes,
                                     std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_minimal_displacement_cost_fn(std::vector<double> initial_guess,
                                       std::vector<double> minimal_displacement_factors) -> CostFn;

auto make_ik_cost_fn(geometry_msgs::msg::Pose pose,
                     kinematics::KinematicsBase::IKCostFn cost_fn,
                     std::shared_ptr<moveit::core::RobotModel const> robot_model,
                     moveit::core::JointModelGroup const* jmg,
                     std::vector<double> initial_guess) -> CostFn;

// Create a solution test function from frame tests and goals
using SolutionTestFn = std::function<bool(std::vector<double> const& active_positions)>;

auto make_is_solution_test_fn(std::vector<FrameTestFn> frame_tests,
                              std::vector<Goal> goals,
                              double cost_threshold,
                              FkFn fk) -> SolutionTestFn;

using FitnessFn = std::function<double(std::vector<double> const& active_positions)>;

auto make_fitness_fn(std::vector<PoseCostFn> pose_cost_functions, std::vector<Goal> goals, FkFn fk)
    -> FitnessFn;

}  // namespace gd_ik
