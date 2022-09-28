#pragma once

#include <memory>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

namespace pick_ik {

auto make_joint_axes(std::shared_ptr<moveit::core::RobotModel const> const& model)
    -> std::vector<tf2::Vector3>;

auto make_link_frames(std::shared_ptr<moveit::core::RobotModel const> const& model)
    -> std::vector<Eigen::Isometry3d>;

auto get_frame(moveit::core::JointModel const& joint_model,
               std::vector<double> const& variables,
               std::vector<tf2::Vector3> const& joint_axes) -> Eigen::Isometry3d;

auto get_frame(moveit::core::LinkModel const& link_model,
               std::vector<Eigen::Isometry3d> const& link_frame) -> Eigen::Isometry3d;

auto has_joint_moved(moveit::core::JointModel const& joint_model,
                     std::vector<double> const& cached_variables,
                     std::vector<double> const& variables) -> bool;

struct CachedJointFrames {
    std::vector<double> variables;
    std::vector<Eigen::Isometry3d> frames;
};

auto get_frame(CachedJointFrames& cache,
               moveit::core::JointModel const& joint_model,
               std::vector<double> const& variables,
               std::vector<tf2::Vector3> const& joint_axes) -> Eigen::Isometry3d;

}  // namespace pick_ik
