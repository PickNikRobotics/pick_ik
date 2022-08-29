#include <gd_ik/forward_kinematics.hpp>
#include <gd_ik/frame.hpp>

#include <algorithm>
#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <vector>

namespace gd_ik {

auto fk_moveit(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    std::vector<size_t> const& tip_link_indexes,
    std::vector<double> const& variables) -> std::vector<Frame> {
  auto robot_state = moveit::core::RobotState{robot_model};

  // force a mimic update
  auto updated_variables = variables;
  robot_model->interpolate(variables.data(), variables.data(), 0.5,
                           updated_variables.data());

  // calculate fk
  robot_state.setVariablePositions(updated_variables);
  robot_state.update();

  std::vector<Frame> tip_frames;
  std::transform(
      tip_link_indexes.cbegin(), tip_link_indexes.cend(),
      std::back_inserter(tip_frames), [&](auto index) {
        auto const* link_model = robot_model->getLinkModel(index);
        return Frame::from(robot_state.getGlobalLinkTransform(link_model));
      });
  return tip_frames;
}

}  // namespace gd_ik
