#pragma once

#include <gd_ik/frame.hpp>

#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <vector>

namespace gd_ik {

auto fk_moveit(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    std::vector<size_t> const& tip_link_indices,
    std::vector<double> const& variables) -> std::vector<Frame>;

}  // namespace gd_ik
