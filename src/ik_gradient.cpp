#include "gd_ik/ik_gradient.hpp"

#include <algorithm>
#include <vector>

#include "gd_ik/fk_moveit.hpp"
#include "gd_ik/frame.hpp"
#include "gd_ik/goal.hpp"

namespace gd_ik {

auto active_variable_positions(
    std::vector<size_t> const& active_variable_indices,
    std::vector<double> const& variables) -> std::vector<double> {
  std::vector<double> active_positions;
  std::transform(active_variable_indices.cbegin(),
                 active_variable_indices.cend(), active_positions.begin(),
                 [&variables](auto index) { return variables.at(index); });
  return active_positions;
}

auto fitness(std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
             std::vector<size_t> const& tip_link_indices,
             std::vector<size_t> const& active_variable_indices,
             std::vector<Goal> const& goals,
             std::vector<double> const& variables) -> double {
  auto tip_frames = fk_moveit(robot_model, tip_link_indices, variables);
  auto active_positions =
      active_variable_positions(active_variable_indices, variables);

  return std::accumulate(
      goals.cbegin(), goals.cend(), 0.0, [&](double sum, auto const& goal) {
        auto const weight_sq = goal.weight * goal.weight;
        auto const fitness =
            goal.eval(tip_frames, active_positions) * weight_sq;
        return sum + fitness;
      });
}

}  // namespace gd_ik
