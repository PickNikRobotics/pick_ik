#pragma once

#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/robot.hpp>

#include <chrono>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <optional>
#include <vector>

namespace gd_ik {

struct GradientIk {
    std::vector<double> gradient;
    std::vector<double> working;
    std::vector<double> local;
    std::vector<double> best;
    double fitness;

    static GradientIk from(std::vector<double> const& initial_guess, FitnessFn const& fitness_fn);
};

auto step(GradientIk& self, Robot const& robot, FitnessFn const& fitness_fn) -> bool;

auto ik_search(std::vector<double> const& initial_guess,
               Robot const& robot,
               FitnessFn const& fitness_fn,
               SolutionTestFn const& solution_fn,
               double timeout) -> std::optional<std::vector<double>>;

}  // namespace gd_ik
