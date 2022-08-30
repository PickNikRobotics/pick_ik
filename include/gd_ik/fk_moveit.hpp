#pragma once

#include <gd_ik/frame.hpp>

#include <functional>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <vector>

namespace gd_ik {

using FkFn = std::function<std::vector<Frame>(std::vector<double> const&)>;

auto make_fk_fn(std::shared_ptr<moveit::core::RobotModel const> robot_model,
                moveit::core::JointModelGroup const* jmg,
                std::vector<size_t> tip_link_indexes) -> FkFn;

}  // namespace gd_ik
