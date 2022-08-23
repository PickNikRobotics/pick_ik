#include "gd_ik/goal.hpp"

#include "gd_ik/frame.hpp"

namespace gd_ik {

auto make_pose_cost_fn(Frame goal, size_t goal_link_index,
                       double rotation_scale) -> CostFn {
  return [=](std::vector<Frame> const& tip_frames,
             std::vector<double> const& active_positions) -> double {
    auto const& frame = tip_frames[goal_link_index];
    return frame.pos.distance2(goal.pos) +
           fmin((frame.rot - goal.rot).length2(),
                (frame.rot + goal.rot).length2()) *
               (rotation_scale * rotation_scale);
  };
}

}  // namespace gd_ik
