#include <gd_ik/algorithm.hpp>
#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/ik_gradient.hpp>
#include <gd_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>

namespace gd_ik {

GradientIk GradientIk::from(std::vector<double> const& initial_guess, FitnessFn const& fitness_fn) {
    return GradientIk{std::vector<double>(initial_guess.size(), 0.0),
                      initial_guess,
                      initial_guess,
                      initial_guess,
                      fitness_fn(initial_guess)};
}

auto step(GradientIk& self,
          Robot const& robot,
          std::vector<size_t> const& active_variable_indexes,
          FitnessFn const& fitness_fn) -> bool {
    double const jd = 0.0001;
    auto const count = self.local.size();
    assert(active_variable_indexes.size() == count);

    // compute gradient direction
    self.working = self.local;
    for (size_t i = 0; i < count; ++i) {
        // test negative displacement
        self.working[i] = self.local[i] - jd;
        double const p1 = fitness_fn(self.working);

        // test positive displacement
        self.working[i] = self.local[i] + jd;
        double const p3 = fitness_fn(self.working);

        // reset self.working
        self.working[i] = self.local[i];

        // + gradient = + on this joint increases fitness fn result
        // - gradient = - on this joint increases fitness fn result
        self.gradient[i] = p3 - p1;
    }

    // normalize gradient direction
    auto sum = std::accumulate(self.gradient.cbegin(),
                               self.gradient.cend(),
                               0.0001,
                               [](auto sum, auto value) { return sum + std::fabs(value); });
    double const f = 1.0 / sum * jd;
    std::transform(self.gradient.cbegin(),
                   self.gradient.cend(),
                   self.gradient.begin(),
                   [&](auto value) { return value * f; });

    // initialize line search
    self.working = self.local;

    for (size_t i = 0; i < count; ++i) {
        self.working[i] = self.local[i] - self.gradient[i];
    }
    double const p1 = fitness_fn(self.working);

    for (size_t i = 0; i < count; ++i) {
        self.working[i] = self.local[i] + self.gradient[i];
    }
    double const p3 = fitness_fn(self.working);
    double const p2 = (p1 + p3) * 0.5;

    // linear step size estimation
    double const cost_diff = (p3 - p1) * 0.5;
    double joint_diff = p2 / cost_diff;

    // if the cost_diff was 0
    if (!isfinite(joint_diff)) joint_diff = 0.0;

    // apply optimization step
    // (move along gradient direction by estimated step size)
    for (size_t i = 0; i < count; ++i) {
        self.working[i] =
            clip(robot, self.local[i] - self.gradient[i] * joint_diff, active_variable_indexes[i]);
    }

    // Always accept the solution and continue
    self.local = self.working;

    // update best solution
    auto const local_cost = fitness_fn(self.local);
    if (local_cost < self.fitness) {
        self.best = self.local;
        self.fitness = local_cost;
        return true;
    }
    return false;
}

auto ik_search(std::vector<double> const& initial_guess,
               Robot const& robot,
               std::vector<size_t> const& active_variable_indexes,
               FitnessFn const& fitness_fn,
               SolutionTestFn const& solution_fn,
               double timeout) -> std::optional<std::vector<double>> {
    if (solution_fn(initial_guess)) {
        return initial_guess;
    }

    auto ik = GradientIk::from(initial_guess, fitness_fn);
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::system_clock::now() < timeout_point) {
        auto const found_better_solution = step(ik, robot, active_variable_indexes, fitness_fn);

        if (found_better_solution) {
            if (solution_fn(ik.best)) {
                return ik.best;
            }
        }
    }

    return std::nullopt;
}

}  // namespace gd_ik
