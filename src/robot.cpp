#include <gd_ik/frame.hpp>
#include <gd_ik/math.hpp>
#include <gd_ik/robot.hpp>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <cfloat>
#include <cmath>
#include <fmt/core.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace gd_ik {

auto Robot::from(std::shared_ptr<moveit::core::RobotModel const> const& model) -> Robot {
    auto robot = Robot{};

    for (auto const& name : model->getVariableNames()) {
        auto const& bounds = model->getVariableBounds(name);

        auto var = Variable{};

        bool bounded = bounds.position_bounded_;

        auto* joint_model = model->getJointOfVariable(robot.variables.size());
        if (dynamic_cast<moveit::core::RevoluteJointModel const*>(joint_model)) {
            if (bounds.max_position_ - bounds.min_position_ >= 2 * M_PI * 0.9999) {
                bounded = false;
            }
        }

        var.min = bounds.min_position_;
        var.max = bounds.max_position_;

        var.clip_min = bounded ? var.min : -DBL_MAX;
        var.clip_max = bounded ? var.max : +DBL_MAX;

        var.span = var.max - var.min;

        if (!(var.span >= 0 && var.span < FLT_MAX)) var.span = 1;

        var.max_velocity = bounds.max_velocity_;
        var.max_velocity_rcp = var.max_velocity > 0.0 ? 1.0 / var.max_velocity : 0.0;

        robot.variables.push_back(var);
    }

    for (size_t ivar = 0; ivar < model->getVariableCount(); ++ivar) {
        robot.variable_joint_types.push_back(model->getJointOfVariable(ivar)->getType());
    }

    return robot;
}

auto get_span(Robot const& self, size_t i) -> double { return self.variables.at(i).span; }

auto get_clip_min(Robot const& self, size_t i) -> double { return self.variables.at(i).clip_min; }

auto get_clip_max(Robot const& self, size_t i) -> double { return self.variables.at(i).clip_max; }

auto get_min(Robot const& self, size_t i) -> double { return self.variables.at(i).min; }

auto get_max(Robot const& self, size_t i) -> double { return self.variables.at(i).max; }

auto is_revolute(Robot const& self, size_t variable_index) -> bool {
    return self.variable_joint_types.at(variable_index) == moveit::core::JointModel::REVOLUTE;
}

auto is_prismatic(Robot const& self, size_t variable_index) -> bool {
    return self.variable_joint_types.at(variable_index) == moveit::core::JointModel::PRISMATIC;
}

auto get_max_velocity(Robot const& self, size_t i) -> double {
    return self.variables.at(i).max_velocity;
}

auto get_max_velocity_rcp(Robot const& self, size_t i) -> double {
    return self.variables.at(i).max_velocity_rcp;
}

auto clip(Robot const& self, double p, size_t i) -> double {
    auto const& info = self.variables.at(i);
    return clamp2(p, info.clip_min, info.clip_max);
}

auto get_link_indexes(std::shared_ptr<moveit::core::RobotModel const> const& model,
                      std::vector<std::string> const& names) -> std::vector<size_t> {
    std::vector<size_t> indexes(names.size(), 0);
    std::transform(names.cbegin(), names.cend(), indexes.begin(), [&model](auto const& name) {
        auto const* link_model = model->getLinkModel(name);
        if (!link_model) {
            throw std::invalid_argument(fmt::format("link not found: {}", name));
        }
        return link_model->getLinkIndex();
    });
    return indexes;
}

auto get_active_variable_indexes(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
                                 moveit::core::JointModelGroup const* jmg,
                                 std::vector<size_t> const& tip_link_indexes)
    -> std::vector<size_t> {
    // Walk the tree of links starting at each tip towards the parent
    // The parent joint of each of these links are the ones we are using
    auto joint_usage = std::vector<int>{};
    joint_usage.resize(robot_model->getJointModelCount(), 0);
    for (auto tip_index : tip_link_indexes) {
        for (auto const* link_model = robot_model->getLinkModels().at(tip_index);
             link_model != nullptr; link_model = link_model->getParentLinkModel()) {
            auto const* joint_model = link_model->getParentJointModel();
            auto const joint_index = joint_model->getJointIndex();
            joint_usage[joint_index] = 1;
        }
    }

    // For each of the active joints in the joint model group
    // If those are in the ones we are using and the joint is not a mimic
    // Then get all the variable names from the joint moodel
    auto active_variable_names = std::set<std::string>{};
    for (auto const* joint_model : jmg->getActiveJointModels()) {
        if (joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic()) {
            for (auto& name : joint_model->getVariableNames()) {
                active_variable_names.insert(name);
            }
        }
    }

    // For each active variable name, add the indexes from that variable to the
    // active variables
    auto active_variable_indexes = std::vector<size_t>{};
    for (auto const& name : active_variable_names) {
        active_variable_indexes.push_back(robot_model->getVariableIndex(name));
    }

    return active_variable_indexes;
}

auto get_minimal_displacement_factors(std::vector<size_t> const& active_variable_indexes,
                                      Robot const& robot) -> std::vector<double> {
    auto minimal_displacement_factors = std::vector<double>{};
    minimal_displacement_factors.resize(active_variable_indexes.size());
    if (double s = std::accumulate(
            active_variable_indexes.cbegin(), active_variable_indexes.cend(), 0.0,
            [&robot](auto sum, auto ivar) { return sum + get_max_velocity_rcp(robot, ivar); });
        s > 0) {
        std::transform(active_variable_indexes.cbegin(), active_variable_indexes.cend(),
                       minimal_displacement_factors.begin(),
                       [&robot, s](auto ivar) { return get_max_velocity_rcp(robot, ivar) / s; });
    } else {
        std::fill(minimal_displacement_factors.begin(), minimal_displacement_factors.end(),
                  1.0 / active_variable_indexes.size());
    }
    return minimal_displacement_factors;
}

auto get_variables(moveit::core::RobotState const& robot_state) -> std::vector<double> {
    auto const count = robot_state.getRobotModel()->getVariableCount();
    auto const* data = robot_state.getVariablePositions();
    std::vector<double> variables(data, data + count);
    return variables;
}

auto transform_poses_to_frames(moveit::core::RobotState const& robot_state,
                               std::vector<geometry_msgs::msg::Pose> const& poses,
                               std::string const& base_frame_name) -> std::vector<Frame> {
    auto frames = std::vector<Frame>{};
    std::transform(poses.cbegin(), poses.cend(), std::back_inserter(frames), [&](auto const& pose) {
        Eigen::Isometry3d p, r;
        tf2::fromMsg(pose, p);
        r = robot_state.getGlobalLinkTransform(base_frame_name);
        return Frame::from(r * p);
    });
    return frames;
}

}  // namespace gd_ik
