#include <pick_ik/robot.hpp>

#include <rsl/random.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tl_expected/expected.hpp>

#include <Eigen/Geometry>
#include <cfloat>
#include <cmath>
#include <fmt/core.h>
#include <limits>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace {
constexpr double kUnboundedVariableHalfSpan = M_PI;
constexpr double kUnboundedJointSampleSpread = M_PI;
}  // namespace

namespace pick_ik {

auto Robot::Variable::generate_valid_value(double init_val /* = 0.0*/) const -> double {
    if (bounded) {
        return rsl::uniform_real(min, max);
    } else {
        return rsl::uniform_real(init_val - kUnboundedJointSampleSpread,
                                 init_val + kUnboundedJointSampleSpread);
    }
}

auto Robot::Variable::is_valid(double val) const -> bool {
    return (!bounded) || (val <= max && val >= min);
}

auto Robot::Variable::clamp_to_limits(double val) const -> double {
    if (bounded) {
        return std::clamp(val, min, max);
    } else {
        return std::clamp(val, val - half_span, val + half_span);
    }
}

auto Robot::from(std::shared_ptr<moveit::core::RobotModel const> const& model,
                 moveit::core::JointModelGroup const* jmg,
                 std::vector<size_t> tip_link_indices) -> Robot {
    auto robot = Robot{};

    auto const active_variable_indices = get_active_variable_indices(model, jmg, tip_link_indices);
    auto const variable_count = active_variable_indices.size();

    auto const& names = model->getVariableNames();

    auto minimal_displacement_divisor = 0.0;

    for (auto ivar : active_variable_indices) {
        auto const& name = names.at(ivar);
        auto const& bounds = model->getVariableBounds(name);

        auto var = Variable{};

        var.bounded = bounds.position_bounded_;
        var.min = bounds.min_position_;
        var.max = bounds.max_position_;
        var.mid = 0.5 * (var.min + var.max);
        var.half_span = var.bounded ? (var.max - var.min) / 2.0 : kUnboundedVariableHalfSpan;

        auto const max_velocity = bounds.max_velocity_;
        var.max_velocity_rcp = max_velocity > 0.0 ? 1.0 / max_velocity : 0.0;

        var.minimal_displacement_factor = 1.0 / static_cast<double>(variable_count);
        minimal_displacement_divisor += var.max_velocity_rcp;

        robot.variables.push_back(var);
    }

    // Calculate minimal displacement factors
    if (minimal_displacement_divisor > 0) {
        for (auto& var : robot.variables) {
            var.minimal_displacement_factor = var.max_velocity_rcp / minimal_displacement_divisor;
        }
    }

    return robot;
}

auto Robot::set_random_valid_configuration(std::vector<double>& config) const -> void {
    auto const num_vars = variables.size();
    if (config.size() != num_vars) {
        config.resize(num_vars);
    }
    for (size_t idx = 0; idx < num_vars; ++idx) {
        config[idx] = variables[idx].generate_valid_value(config[idx]);
    }
}

auto Robot::is_valid_configuration(std::vector<double> const& config) const -> bool {
    auto const num_vars = variables.size();
    for (size_t idx = 0; idx < num_vars; ++idx) {
        if (!variables[idx].is_valid(config[idx])) {
            return false;
        }
    }
    return true;
}

auto get_link_indices(std::shared_ptr<moveit::core::RobotModel const> const& model,
                      std::vector<std::string> const& names)
    -> tl::expected<std::vector<size_t>, std::string> {
    auto indices = std::vector<size_t>();
    for (auto const& name : names) {
        auto const* link_model = model->getLinkModel(name);
        if (!link_model) {
            return tl::make_unexpected(fmt::format("link not found: {}", name));
        }
        indices.push_back(link_model->getLinkIndex());
    }

    return indices;
}

auto get_active_variable_indices(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
                                 moveit::core::JointModelGroup const* jmg,
                                 std::vector<size_t> const& tip_link_indices)
    -> std::vector<size_t> {
    // Walk the tree of links starting at each tip towards the parent
    // The parent joint of each of these links are the ones we are using
    auto joint_usage = std::vector<int>{};
    joint_usage.resize(robot_model->getJointModelCount(), 0);
    for (auto tip_index : tip_link_indices) {
        for (auto const* link_model = robot_model->getLinkModels().at(tip_index);
             link_model != nullptr;
             link_model = link_model->getParentLinkModel()) {
            auto const* joint_model = link_model->getParentJointModel();
            auto const joint_index = joint_model->getJointIndex();
            joint_usage[joint_index] = 1;
        }
    }

    // For each of the active joints in the joint model group
    // If those are in the ones we are using and the joint is not a mimic
    // Then get all the variable names from the joint moodel
    auto active_variable_names = std::vector<std::string>{};
    for (auto const* joint_model : jmg->getActiveJointModels()) {
        if (joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic()) {
            for (auto& name : joint_model->getVariableNames()) {
                active_variable_names.push_back(name);
            }
        }
    }

    // For each active variable name, add the indices from that variable to the
    // active variables
    auto active_variable_indices = std::vector<size_t>{};
    for (auto const& name : active_variable_names) {
        active_variable_indices.push_back(robot_model->getVariableIndex(name));
    }

    return active_variable_indices;
}

auto get_variables(moveit::core::RobotState const& robot_state) -> std::vector<double> {
    auto const count = robot_state.getRobotModel()->getVariableCount();
    auto const* data = robot_state.getVariablePositions();
    std::vector<double> variables(data, data + count);
    return variables;
}

auto transform_poses_to_frames(moveit::core::RobotState const& robot_state,
                               std::vector<geometry_msgs::msg::Pose> const& poses,
                               std::string const& base_frame_name)
    -> std::vector<Eigen::Isometry3d> {
    auto frames = std::vector<Eigen::Isometry3d>{};
    std::transform(poses.cbegin(), poses.cend(), std::back_inserter(frames), [&](auto const& pose) {
        Eigen::Isometry3d p;
        tf2::fromMsg(pose, p);
        auto const r = robot_state.getGlobalLinkTransform(base_frame_name);
        return (r * p);
    });
    return frames;
}

}  // namespace pick_ik
