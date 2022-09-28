#pragma once

#include <tl_expected/expected.hpp>

#include <Eigen/Geometry>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <string>
#include <vector>

namespace pick_ik {

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
                     std::vector<size_t> tip_link_indices) -> Robot;
};

auto get_link_indices(std::shared_ptr<moveit::core::RobotModel const> const& model,
                      std::vector<std::string> const& names)
    -> tl::expected<std::vector<size_t>, std::string>;

auto get_active_variable_indices(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
                                 moveit::core::JointModelGroup const* jmg,
                                 std::vector<size_t> const& tip_link_indices)
    -> std::vector<size_t>;

auto get_minimal_displacement_factors(std::vector<size_t> const& active_variable_indices,
                                      Robot const& robot) -> std::vector<double>;

auto get_variables(moveit::core::RobotState const& robot_state) -> std::vector<double>;

auto transform_poses_to_frames(moveit::core::RobotState const& robot_state,
                               std::vector<geometry_msgs::msg::Pose> const& poses,
                               std::string const& base_frame_name)
    -> std::vector<Eigen::Isometry3d>;

}  // namespace pick_ik
