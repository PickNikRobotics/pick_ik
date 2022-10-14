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

struct GradientIkParams {
    double step_size = 0.0001;        // Step size for gradient descent.
    double min_cost_delta = 1.0e-12;  // Minimum cost difference for termination.
    double max_time = 0.05;           // Maximum time elapsed for termination.
    int max_iterations = 100;         // Maximum iterations for termination.
};

struct GradientIk {
    std::vector<double> gradient;
    std::vector<double> working;
    std::vector<double> local;
    std::vector<double> best;
    double local_cost;
    double best_cost;

    static GradientIk from(std::vector<double> const& initial_guess, CostFn const& cost_fn);
};

/// Performs one step of gradient descent.
/// @param self Instance of GradientIk object.
/// @param robot Robot model,
/// @param cost_fn Cost function for gradient descent.
/// @param step_size Numerical step size for gradient descent.
/// @return true if the cost function improved (decreased), else false.
auto step(GradientIk& self, Robot const& robot, CostFn const& cost_fn, double step_size) -> bool;

auto ik_gradient(std::vector<double> const& initial_guess,
                 Robot const& robot,
                 CostFn const& cost_fn,
                 SolutionTestFn const& solution_fn,
                 GradientIkParams const& params,
                 bool approx_solution) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
