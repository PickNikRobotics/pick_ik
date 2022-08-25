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

auto make_frame_test_fn(Frame goal_frame, double position_threshold,
                        double rotation_threshold, double twist_threshold)
    -> FrameTestFn {
  return [=](Frame const& tip_frame) -> bool {
    if (position_threshold != DBL_MAX || rotation_threshold != DBL_MAX) {
      double p_dist = (tip_frame.pos - goal_frame.pos).length();
      double r_dist = tip_frame.rot.angleShortestPath(goal_frame.rot);
      r_dist = r_dist * 180 / M_PI;
      if (!(p_dist <= position_threshold)) return false;
      if (!(r_dist <= rotation_threshold)) return false;
    }
    if (twist_threshold != DBL_MAX) {
      auto goal_kdl = to_KDL(goal_frame);
      auto tip_kdl = to_KDL(tip_frame);
      KDL::Twist kdl_diff(
          goal_kdl.M.Inverse() * KDL::diff(goal_kdl.p, tip_kdl.p),
          goal_kdl.M.Inverse() * KDL::diff(goal_kdl.M, tip_kdl.M));
      if (!KDL::Equal(kdl_diff, KDL::Twist::Zero(), twist_threshold))
        return false;
    }
    return true;
  };
}

auto make_frame_tests(std::vector<Frame> goal_frames, double position_threshold,
                      double rotation_threshold, double twist_threshold)
    -> std::vector<FrameTestFn> {
  auto tests = std::vector<FrameTestFn>{};
  std::transform(goal_frames.cbegin(), goal_frames.cend(), tests.begin(),
                 [&](auto const& frame) {
                   return make_frame_test_fn(frame, position_threshold,
                                             rotation_threshold,
                                             twist_threshold);
                 });
  return tests;
}

auto make_pose_cost_fn(Frame goal, size_t goal_link_index,
                       double rotation_scale) -> PoseCostFn {
  return [=](std::vector<Frame> const& tip_frames) -> double {
    auto const& frame = tip_frames[goal_link_index];
    return frame.pos.distance2(goal.pos) +
           fmin((frame.rot - goal.rot).length2(),
                (frame.rot + goal.rot).length2()) *
               (rotation_scale * rotation_scale);
  };
}

auto make_pose_cost_functions(std::vector<Frame> goal_frames,
                              double rotation_scale)
    -> std::vector<PoseCostFn> {
  auto cost_functions = std::vector<PoseCostFn>{};
  for (size_t i = 0; i < goal_frames.size(); ++i) {
    cost_functions.push_back(
        make_pose_cost_fn(goal_frames[i], i, rotation_scale));
  }
  return cost_functions;
}

auto make_center_joints_cost_fn(
    Robot robot, std::vector<size_t> active_variable_indexes,
    std::vector<double> minimal_displacement_factors) -> CostFn {
  return [=](std::vector<double> const& active_positions) -> double {
    double sum = 0;
    assert(active_positions.size() == active_variable_indexes.size() &&
           active_positions.size() == minimal_displacement_factors.size());
    for (size_t i = 0; i < active_positions.size(); ++i) {
      auto const index = active_variable_indexes[i];
      if (get_clip_max(robot, index) == DBL_MAX) {
        continue;
      }

      auto const position = active_positions[i];
      auto const weight = minimal_displacement_factors[i];
      auto const min = get_min(robot, index);
      auto const max = get_max(robot, index);
      auto const mid = (min + max) * 0.5;
      sum += std::pow((position - mid) * weight, 2);
    }
    return sum;
  };
}

auto make_avoid_joint_limits_cost_fn(
    Robot robot, std::vector<size_t> active_variable_indexes,
    std::vector<double> minimal_displacement_factors) -> CostFn {
  return [=](std::vector<double> const& active_positions) -> double {
    double sum = 0;
    assert(active_positions.size() == active_variable_indexes.size() &&
           active_positions.size() == minimal_displacement_factors.size());
    for (size_t i = 0; i < active_positions.size(); ++i) {
      auto const index = active_variable_indexes[i];
      if (get_clip_max(robot, index) == DBL_MAX) {
        continue;
      }

      auto const position = active_positions[i];
      auto const weight = minimal_displacement_factors[i];
      auto const min = get_min(robot, index);
      auto const max = get_max(robot, index);
      auto const mid = (min + max) * 0.5;
      auto const span = get_span(robot, index);
      sum += std::pow(
          std::fmax(0.0, std::fabs(position - mid) * 2.0 - span * 0.5) * weight,
          2);
    }
    return sum;
  };
}

auto make_minimal_displacement_cost_fn(
    std::vector<double> initial_guess,
    std::vector<double> minimal_displacement_factors) -> CostFn {
  return [=](std::vector<double> const& active_positions) -> double {
    double sum = 0;
    assert(active_positions.size() == initial_guess.size() &&
           active_positions.size() == minimal_displacement_factors.size());
    for (size_t i = 0; i < active_positions.size(); ++i) {
      auto const guess = initial_guess[i];
      auto const position = active_positions[i];
      auto const weight = minimal_displacement_factors[i];
      sum += std::pow((position - guess) * weight, 2);
    }
    return sum;
  };
}

auto make_ik_cost_fn(
    geometry_msgs::msg::Pose pose, kinematics::KinematicsBase::IKCostFn cost_fn,
    std::shared_ptr<moveit::core::RobotModel const> robot_model,
    moveit::core::JointModelGroup const* jmg, std::vector<double> initial_guess)
    -> CostFn {
  return [=](std::vector<double> const& active_positions) -> double {
    auto robot_state = moveit::core::RobotState(robot_model);
    robot_state.setJointGroupPositions(jmg, active_positions);
    robot_state.update();
    return cost_fn(pose, robot_state, jmg, initial_guess);
  };
}

auto make_is_solution_test_fn(std::vector<FrameTestFn> frame_tests,
                              std::vector<Goal> goals, double cost_threshold)
    -> SolutionTestFn {
  return [=](std::vector<Frame> const& tip_frames,
             std::vector<double> const& active_positions) {
    assert(frame_tests.size() == tip_frames.size());
    for (size_t i = 0; i < frame_tests.size(); ++i) {
      if (!frame_tests[i](tip_frames[i])) {
        return false;
      }
    }

    for (auto const& goal : goals) {
      auto const cost = goal.eval(active_positions) * std::pow(goal.weight, 2);
      if (cost >= std::pow(cost_threshold, 2)) {
        return false;
      }
    }

    return true;
  };
}

auto make_fitness_fn(std::vector<PoseCostFn> pose_cost_functions,
                     std::vector<Goal> goals) -> FitnessFn {
  return [=](std::vector<Frame> const& tip_frames,
             std::vector<double> const& active_positions) {
    return std::accumulate(
        goals.cbegin(), goals.cend(),
        std::accumulate(
            pose_cost_functions.cbegin(), pose_cost_functions.cend(), 0.0,
            [&](auto sum, auto const& fn) { return sum + fn(tip_frames); }),
        [&](auto sum, auto const& goal) {
          return sum + goal.eval(active_positions) * std::pow(goal.weight, 2);
        });
  };
}

}  // namespace gd_ik
