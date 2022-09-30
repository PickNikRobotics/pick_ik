#pragma once

#include <pick_ik/goal.hpp>
#include <pick_ik/robot.hpp>

#include <chrono>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <optional>
#include <vector>

namespace pick_ik {

struct MemeticIk {
    std::vector<double> best;
    double cost;

    static MemeticIk from(std::vector<double> const& initial_guess, CostFn const& cost_fn);
};

// auto step(MemeticIk& self, Robot const& robot, CostFn const& cost_fn) -> bool;

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                double timeout = 10.0,
                bool approx_solution = false) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
