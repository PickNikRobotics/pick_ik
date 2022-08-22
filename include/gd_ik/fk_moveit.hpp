#pragma once

#include <moveit/robot_model/robot_model.h>

#include <memory>
#include <vector>

#include "gd_ik/frame.hpp"

namespace gd_ik {

auto fk_moveit(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    std::vector<size_t> const& tip_link_indices,
    std::vector<double> const& variables) -> std::vector<Frame>;

}  // namespace gd_ik
