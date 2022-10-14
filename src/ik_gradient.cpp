#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <optional>
#include <vector>

namespace pick_ik {

GradientIk GradientIk::from(std::vector<double> const& initial_guess, CostFn const& cost_fn) {
    auto const initial_cost = cost_fn(initial_guess);
    return GradientIk{std::vector<double>(initial_guess.size(), 0.0),
                      initial_guess,
                      initial_guess,
                      initial_guess,
                      initial_cost,
                      initial_cost};
}

auto step(GradientIk& self, Robot const& robot, CostFn const& cost_fn, double step_size) -> bool {
    auto const count = self.local.size();

    // compute gradient direction
    self.working = self.local;
    for (size_t i = 0; i < count; ++i) {
        // test negative displacement
        self.working[i] = self.local[i] - step_size;
        double const p1 = cost_fn(self.working);

        // test positive displacement
        self.working[i] = self.local[i] + step_size;
        double const p3 = cost_fn(self.working);

        // reset self.working
        self.working[i] = self.local[i];

        // + gradient = + on this joint increases cost fn result
        // - gradient = - on this joint increases cost fn result
        self.gradient[i] = p3 - p1;
    }

    // normalize gradient direction
    auto sum = std::accumulate(self.gradient.cbegin(),
                               self.gradient.cend(),
                               step_size,
                               [](auto acc, auto value) { return acc + std::fabs(value); });
    double const f = 1.0 / sum * step_size;
    std::transform(self.gradient.cbegin(),
                   self.gradient.cend(),
                   self.gradient.begin(),
                   [&](auto value) { return value * f; });

    // initialize line search
    self.working = self.local;

    for (size_t i = 0; i < count; ++i) {
        self.working[i] = self.local[i] - self.gradient[i];
    }
    double const p1 = cost_fn(self.working);

    for (size_t i = 0; i < count; ++i) {
        self.working[i] = self.local[i] + self.gradient[i];
    }
    double const p3 = cost_fn(self.working);
    double const p2 = (p1 + p3) * 0.5;

    // linear step size estimation
    double const cost_diff = (p3 - p1) * 0.5;
    double joint_diff = p2 / cost_diff;

    // if the cost_diff was 0
    if (!isfinite(joint_diff)) joint_diff = 0.0;

    // apply optimization step
    // (move along gradient direction by estimated step size)
    for (size_t i = 0; i < count; ++i) {
        self.working[i] = std::clamp(self.local[i] - self.gradient[i] * joint_diff,
                                     robot.variables[i].clip_min,
                                     robot.variables[i].clip_max);
    }

    // Always accept the solution and continue
    self.local = self.working;
    self.local_cost = cost_fn(self.local);

    // Update best solution
    if (self.local_cost < self.best_cost) {
        self.best = self.local;
        self.best_cost = self.local_cost;
        return true;
    }
    return false;
}

auto ik_gradient(std::vector<double> const& initial_guess,
                 Robot const& robot,
                 CostFn const& cost_fn,
                 SolutionTestFn const& solution_fn,
                 GradientIkParams const& params,
                 bool approx_solution) -> std::optional<std::vector<double>> {
    if (solution_fn(initial_guess)) {
        return initial_guess;
    }

    assert(robot.variables.size() == initial_guess.size());
    auto ik = GradientIk::from(initial_guess, cost_fn);

    // Main loop
    int num_iterations = 0;
    double previous_cost = 0.0;
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(params.max_time);

    while ((std::chrono::system_clock::now() < timeout_point) &&
           (num_iterations < params.max_iterations)) {
        if (step(ik, robot, cost_fn, params.step_size)) {
            if (solution_fn(ik.best)) {
                return ik.best;
            }
        }

        if (abs(ik.local_cost - previous_cost) <= params.min_cost_delta) {
            break;
        }
        previous_cost = ik.local_cost;
        num_iterations++;
    }

    // If no solution was found, either return the approximate solution or nothing.
    if (approx_solution) {
        return ik.best;
    }
    return std::nullopt;
}

}  // namespace pick_ik
