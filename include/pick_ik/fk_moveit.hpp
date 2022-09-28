#pragma once

#include <Eigen/Geometry>
#include <functional>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <vector>

namespace pick_ik {

using FkFn = std::function<std::vector<Eigen::Isometry3d>(std::vector<double> const&)>;

auto make_fk_fn(std::shared_ptr<moveit::core::RobotModel const> robot_model,
                moveit::core::JointModelGroup const* jmg,
                std::vector<size_t> tip_link_indexes) -> FkFn;

}  // namespace pick_ik
