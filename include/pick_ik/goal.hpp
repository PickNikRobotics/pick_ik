#pragma once

#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/robot.hpp>

#include <Eigen/Geometry>
#include <functional>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <string>
#include <vector>

namespace pick_ik {

// Frame equality tests
using FrameTestFn = std::function<bool(Eigen::Isometry3d const& tip_frame)>;
auto make_frame_tests(std::vector<Eigen::Isometry3d> goal_frames,
                      double position_threshold,
                      std::optional<double> orientation_threshold = std::nullopt)
    -> std::vector<FrameTestFn>;

// Pose cost functions
using PoseCostFn = std::function<double(std::vector<Eigen::Isometry3d> const& tip_frames)>;
auto make_pose_cost_fn(Eigen::Isometry3d goal, size_t goal_link_index, double rotation_scale)
    -> PoseCostFn;

auto make_pose_cost_functions(std::vector<Eigen::Isometry3d> goal_frames, double rotation_scale)
    -> std::vector<PoseCostFn>;

// Goal Function type
using CostFn = std::function<double(std::vector<double> const& active_positions)>;

struct Goal {
    CostFn eval;
    double weight;
};

auto make_center_joints_cost_fn(Robot robot) -> CostFn;

auto make_avoid_joint_limits_cost_fn(Robot robot) -> CostFn;

auto make_minimal_displacement_cost_fn(Robot robot, std::vector<double> initial_guess) -> CostFn;

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

using CostFn = std::function<double(std::vector<double> const& active_positions)>;

auto make_cost_fn(std::vector<PoseCostFn> pose_cost_functions, std::vector<Goal> goals, FkFn fk)
    -> CostFn;

}  // namespace pick_ik
