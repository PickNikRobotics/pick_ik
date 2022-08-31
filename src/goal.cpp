#include <gd_ik/algorithm.hpp>
#include <gd_ik/fk_moveit.hpp>
#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/robot.hpp>

#include <algorithm>
#include <cmath>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>
#include <vector>

namespace gd_ik {

auto make_frame_test_fn(Frame goal_frame, double twist_threshold) -> FrameTestFn {
    return [=](Frame const& tip_frame) -> bool {
        auto goal_kdl = to_KDL(goal_frame);
        auto tip_kdl = to_KDL(tip_frame);
        KDL::Twist kdl_diff(goal_kdl.M.Inverse() * KDL::diff(goal_kdl.p, tip_kdl.p),
                            goal_kdl.M.Inverse() * KDL::diff(goal_kdl.M, tip_kdl.M));
        if (!KDL::Equal(kdl_diff, KDL::Twist::Zero(), twist_threshold)) {
            return false;
        }
        return true;
    };
}

auto make_frame_tests(std::vector<Frame> goal_frames, double twist_threshold)
    -> std::vector<FrameTestFn> {
    auto tests = std::vector<FrameTestFn>{};
    std::transform(goal_frames.cbegin(),
                   goal_frames.cend(),
                   std::back_inserter(tests),
                   [&](auto const& frame) { return make_frame_test_fn(frame, twist_threshold); });
    return tests;
}

auto make_pose_cost_fn(Frame goal, size_t goal_link_index, double rotation_scale) -> PoseCostFn {
    if (rotation_scale > 0.0) {
        return [=](std::vector<Frame> const& tip_frames) -> double {
            auto const& frame = tip_frames[goal_link_index];
            return frame.pos.distance2(goal.pos) +
                   fmin((frame.rot - goal.rot).length2(), (frame.rot + goal.rot).length2()) *
                       (rotation_scale * rotation_scale);
        };
    }
    return [=](std::vector<Frame> const& tip_frames) -> double {
        auto const& frame = tip_frames[goal_link_index];
        return frame.pos.distance2(goal.pos);
    };
}

auto make_pose_cost_functions(std::vector<Frame> goal_frames, double rotation_scale)
    -> std::vector<PoseCostFn> {
    auto cost_functions = std::vector<PoseCostFn>{};
    for (size_t i = 0; i < goal_frames.size(); ++i) {
        cost_functions.push_back(make_pose_cost_fn(goal_frames[i], i, rotation_scale));
    }
    return cost_functions;
}

auto make_center_joints_cost_fn(Robot robot) -> CostFn {
    return [=](std::vector<double> const& active_positions) -> double {
        double sum = 0;
        assert(robot.variables.size() == active_positions.size());
        for (size_t i = 0; i < active_positions.size(); ++i) {
            auto const& variable = robot.variables[i];
            if (variable.clip_max == DBL_MAX) {
                continue;
            }

            auto const position = active_positions[i];
            auto const weight = variable.minimal_displacement_factor;
            auto const mid = (variable.min + variable.max) * 0.5;
            sum += std::pow((position - mid) * weight, 2);
        }
        return sum;
    };
}

auto make_avoid_joint_limits_cost_fn(Robot robot) -> CostFn {
    return [=](std::vector<double> const& active_positions) -> double {
        double sum = 0;
        assert(robot.variables.size() == active_positions.size());
        for (size_t i = 0; i < active_positions.size(); ++i) {
            auto const& variable = robot.variables[i];
            if (variable.clip_max == DBL_MAX) {
                continue;
            }

            auto const position = active_positions[i];
            auto const weight = variable.minimal_displacement_factor;
            auto const mid = (variable.min + variable.max) * 0.5;
            auto const span = variable.span;
            sum +=
                std::pow(std::fmax(0.0, std::fabs(position - mid) * 2.0 - span * 0.5) * weight, 2);
        }
        return sum;
    };
}

auto make_minimal_displacement_cost_fn(Robot robot, std::vector<double> initial_guess) -> CostFn {
    return [=](std::vector<double> const& active_positions) -> double {
        double sum = 0;
        assert(active_positions.size() == robot.variables.size() &&
               active_positions.size() == initial_guess.size());
        for (size_t i = 0; i < active_positions.size(); ++i) {
            auto const guess = initial_guess[i];
            auto const position = active_positions[i];
            auto const weight = robot.variables[i].minimal_displacement_factor;
            sum += std::pow((position - guess) * weight, 2);
        }
        return sum;
    };
}

auto make_ik_cost_fn(geometry_msgs::msg::Pose pose,
                     kinematics::KinematicsBase::IKCostFn cost_fn,
                     std::shared_ptr<moveit::core::RobotModel const> robot_model,
                     moveit::core::JointModelGroup const* jmg,
                     std::vector<double> initial_guess) -> CostFn {
    return [=](std::vector<double> const& active_positions) -> double {
        auto robot_state = moveit::core::RobotState(robot_model);
        robot_state.setJointGroupPositions(jmg, active_positions);
        robot_state.update();
        return cost_fn(pose, robot_state, jmg, initial_guess);
    };
}

auto make_is_solution_test_fn(std::vector<FrameTestFn> frame_tests,
                              std::vector<Goal> goals,
                              double cost_threshold,
                              FkFn fk) -> SolutionTestFn {
    return [=](std::vector<double> const& active_positions) {
        auto tip_frames = fk(active_positions);
        assert(frame_tests.size() == tip_frames.size());
        for (size_t i = 0; i < frame_tests.size(); ++i) {
            if (!frame_tests[i](tip_frames[i])) {
                return false;
            }
        }

        auto const cost_threshold_sq = std::pow(cost_threshold, 2);
        for (auto const& goal : goals) {
            auto const cost = goal.eval(active_positions) * std::pow(goal.weight, 2);
            if (cost >= cost_threshold_sq) {
                return false;
            }
        }

        return true;
    };
}

auto make_fitness_fn(std::vector<PoseCostFn> pose_cost_functions, std::vector<Goal> goals, FkFn fk)
    -> FitnessFn {
    return [=](std::vector<double> const& active_positions) {
        auto tip_frames = fk(active_positions);
        auto const pose_cost =
            std::accumulate(pose_cost_functions.cbegin(),
                            pose_cost_functions.cend(),
                            0.0,
                            [&](auto sum, auto const& fn) { return sum + fn(tip_frames); });
        auto const goal_cost =
            std::accumulate(goals.cbegin(), goals.cend(), 0.0, [&](auto sum, auto const& goal) {
                return sum + goal.eval(active_positions) * std::pow(goal.weight, 2);
            });
        return pose_cost + goal_cost;
    };
}

}  // namespace gd_ik
