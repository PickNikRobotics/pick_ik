#pragma once

#include <gd_ik/frame.hpp>

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <vector>

namespace gd_ik {

struct Robot {
    struct Variable {
        double clip_min, clip_max;
        double span;
        double min;
        double max;
        double max_velocity_rcp;
        double minimal_displacement_factor;
    };
    std::vector<Variable> variables;

    // Create new Robot from a RobotModel
    static auto from(std::shared_ptr<moveit::core::RobotModel const> const& model,
                     moveit::core::JointModelGroup const* jmg,
                     std::vector<size_t> tip_link_indexes) -> Robot;
};

auto get_link_indexes(std::shared_ptr<moveit::core::RobotModel const> const& model,
                      std::vector<std::string> const& names) -> std::vector<size_t>;

auto get_active_variable_indexes(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
                                 moveit::core::JointModelGroup const* jmg,
                                 std::vector<size_t> const& tip_link_indexes)
    -> std::vector<size_t>;

auto get_minimal_displacement_factors(std::vector<size_t> const& active_variable_indexes,
                                      Robot const& robot) -> std::vector<double>;

auto get_variables(moveit::core::RobotState const& robot_state) -> std::vector<double>;

auto transform_poses_to_frames(moveit::core::RobotState const& robot_state,
                               std::vector<geometry_msgs::msg::Pose> const& poses,
                               std::string const& base_frame_name) -> std::vector<Frame>;

}  // namespace gd_ik
