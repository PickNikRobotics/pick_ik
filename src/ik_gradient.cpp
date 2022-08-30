#include <gd_ik/algorithm.hpp>
#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/ik_gradient.hpp>
#include <gd_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <optional>
#include <vector>

namespace gd_ik {

auto step(GradientIk& self, Robot const& robot,
          std::vector<size_t> const& active_variable_indexes,
          FitnessFn const& fitness_fn) -> bool {
  auto gradient = std::vector<double>(self.local.size(), 0.0);
  auto temp = self.local;
  double const jd = 0.0001;
  auto const count = gradient.size();

  assert(active_variable_indexes.size() == count);

  // compute gradient direction
  for (size_t i = 0; i < count; ++i) {
    temp[i] = self.local[i] - jd;
    double const p1 = fitness_fn(temp);

    temp[i] = self.local[i] + jd;
    double const p3 = fitness_fn(temp);

    temp[i] = self.local[i];

    gradient[i] = p3 - p1;
  }

  // normalize gradient direction
  auto sum = std::accumulate(
      gradient.cbegin(), gradient.cend(), 0.0001,
      [](auto sum, auto value) { return sum + std::fabs(value); });
  double const f = 1.0 / sum * jd;
  std::transform(gradient.cbegin(), gradient.cend(), gradient.begin(),
                 [&](auto value) { return value * f; });

  // initialize line search
  temp = self.local;

  for (size_t i = 0; i < count; ++i) {
    temp[i] = self.local[i] - gradient[i];
  }
  double const p1 = fitness_fn(temp);

  for (size_t i = 0; i < count; ++i) {
    temp[i] = self.local[i] + gradient[i];
  }
  double const p3 = fitness_fn(temp);

  double const p2 = (p1 + p3) * 0.5;

  // linear step size estimation
  double const cost_diff = (p3 - p1) * 0.5;
  double joint_diff = p2 / cost_diff;

  // if the cost_diff was not 0
  if (std::isfinite(joint_diff)) {
    // apply optimization step
    // (move along gradient direction by estimated step size)
    for (size_t i = 0; i < count; ++i) {
      temp[i] = clip(robot, self.local[i] - gradient[i] * joint_diff,
                     active_variable_indexes[i]);
    }
  }

  auto const local_fitness = fitness_fn(self.local);
  auto const temp_fitness = fitness_fn(temp);

  // has solution improved?
  if (temp_fitness < local_fitness) {
    // solution improved -> accept solution
    self.local = temp;
  }

  // update best solution
  if (local_fitness < self.fitness) {
    self.best = self.local;
    self.fitness = local_fitness;
    return true;
  }
  return false;
}

auto ik_search(std::vector<double> const& initial_guess, Robot const& robot,
               std::vector<size_t> const& active_variable_indexes,
               FitnessFn const& fitness_fn, SolutionTestFn const& solution_fn,
               double timeout) -> std::optional<std::vector<double>> {
  auto ik = GradientIk{initial_guess, initial_guess, DBL_MAX};
  fmt::print(stderr, "{}: {}\n", ik.fitness, fmt::join(ik.best, ", "));

  timeout = 10.0;

  auto const timeout_point =
      std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
  while (std::chrono::system_clock::now() < timeout_point) {
    auto const found_better_solution =
        step(ik, robot, active_variable_indexes, fitness_fn);

    if (found_better_solution) {
      fmt::print(stderr, "{}: {}\n", ik.fitness, fmt::join(ik.best, ", "));
      if (solution_fn(ik.best)) {
        return ik.best;
      }
    }
  }

  return std::nullopt;
}

}  // namespace gd_ik
