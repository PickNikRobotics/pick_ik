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
        /// @brief Min, max, and middle position values of the variable.
        double min, max, mid;

        /// @brief Whether the variable's position is bounded.
        bool bounded;

        /// @brief The half-span (min - max) / 2.0 of the variable, or a default value if unbounded.
        double half_span;

        double max_velocity_rcp;
        double minimal_displacement_factor;

        /// @brief Generates a valid variable value given an optional initial value (for unbounded
        /// joints).
        auto generate_valid_value(double init_val = 0.0) const -> double;

        /// @brief Returns true if a value is valid given the variable bounds.
        auto is_valid(double val) const -> bool;

        /// @brief Clamps a configuration to joint limits.
        auto clamp_to_limits(double val) const -> double;
    };
    std::vector<Variable> variables;

    /** @brief Create new Robot from a RobotModel. */
    static auto from(std::shared_ptr<moveit::core::RobotModel const> const& model,
                     moveit::core::JointModelGroup const* jmg,
                     std::vector<size_t> tip_link_indices) -> Robot;

    /**
     * @brief Sets a variable vector to a random configuration.
     * @details Here, "valid" denotes that the joint values are with their specified limits.
     */
    auto set_random_valid_configuration(std::vector<double>& config) const -> void;

    /** @brief Check is a configuration is valid. */
    auto is_valid_configuration(std::vector<double> const& config) const -> bool;
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
