#include <pick_ik/fk_moveit.hpp>

#include <algorithm>
#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>

namespace pick_ik {

auto make_fk_fn(std::shared_ptr<moveit::core::RobotModel const> robot_model,
                moveit::core::JointModelGroup const* jmg,
                std::mutex& mx,
                std::vector<size_t> tip_link_indices) -> FkFn {
    auto robot_state = moveit::core::RobotState(robot_model);
    robot_state.setToDefaultValues();

    // IK function is mutable so it re-uses the robot_state instead of creating
    // new copies. This function accepts a mutex so that it can be made thread-safe.
    return [=, &mx](std::vector<double> const& active_positions) mutable {
        std::scoped_lock lock(mx);
        robot_state.setJointGroupPositions(jmg, active_positions);
        robot_state.updateLinkTransforms();

        std::vector<Eigen::Isometry3d> tip_frames;
        std::transform(tip_link_indices.cbegin(),
                       tip_link_indices.cend(),
                       std::back_inserter(tip_frames),
                       [&](auto index) {
                           auto const* link_model = robot_model->getLinkModel(index);
                           return robot_state.getGlobalLinkTransform(link_model);
                       });
        return tip_frames;
    };
}

}  // namespace pick_ik
