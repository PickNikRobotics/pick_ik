#include <gd_ik/fk_moveit.hpp>
#include <gd_ik/frame.hpp>

#include <algorithm>
#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>

namespace gd_ik {

auto make_fk_fn(std::shared_ptr<moveit::core::RobotModel const> robot_model,
                moveit::core::JointModelGroup const* jmg, std::vector<size_t> tip_link_indexes)
    -> FkFn {
    return [=](std::vector<double> const& active_positions) {
        auto robot_state = moveit::core::RobotState(robot_model);
        robot_state.setToDefaultValues();
        robot_state.setJointGroupPositions(jmg, active_positions);
        robot_state.updateLinkTransforms();

        std::vector<Frame> tip_frames;
        std::transform(tip_link_indexes.cbegin(), tip_link_indexes.cend(),
                       std::back_inserter(tip_frames), [&](auto index) {
                           auto const* link_model = robot_model->getLinkModel(index);
                           return Frame::from(robot_state.getGlobalLinkTransform(link_model));
                       });
        return tip_frames;
    };
}

}  // namespace gd_ik
