#pragma once

#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_model/robot_model.h>
#include <tf2/LinearMath/Vector3.h>

#include <memory>
#include <vector>

#include "gd_ik/frame.hpp"

namespace gd_ik {

std::vector<tf2::Vector3> make_joint_axes(
    std::shared_ptr<moveit::core::RobotModel const> const& model);

std::vector<Frame> make_link_frames(
    std::shared_ptr<moveit::core::RobotModel const> const& model);

Frame get_frame(moveit::core::JointModel const& joint_model,
                std::vector<double> const& variables,
                std::vector<tf2::Vector3> const& joint_axes);

Frame get_frame(moveit::core::LinkModel const& link_model,
                std::vector<Frame> const& link_frame);

bool has_joint_moved(moveit::core::JointModel const& joint_model,
                     std::vector<double> const& cached_variables,
                     std::vector<double> const& variables);

struct CachedJointFrames {
  std::vector<double> variables;
  std::vector<Frame> frames;
};
Frame get_frame(CachedJointFrames& cache,
                moveit::core::JointModel const& joint_model,
                std::vector<double> const& variables,
                std::vector<tf2::Vector3> const& joint_axes);

}  // namespace gd_ik
