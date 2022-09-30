#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <random>
#include <vector>

namespace pick_ik {

MemeticIk MemeticIk::from(std::vector<double> const& initial_guess, CostFn const& cost_fn) {
    return MemeticIk{std::vector<double>(initial_guess.size(), 0.0), cost_fn(initial_guess)};
}

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                double timeout,
                bool approx_solution) -> std::optional<std::vector<double>> {
    if (solution_fn(initial_guess)) {
        return initial_guess;
    }

    assert(robot.variables.size() == initial_guess.size());
    auto ik = MemeticIk::from(initial_guess, cost_fn);

    size_t constexpr population_size = 8;  // Placeholder
    // Randomize a population.
    std::vector<std::vector<double>> population;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(-0.5, 0.5);
    population.reserve(population_size);
    for (size_t i = 0; i < population_size; ++i) {
        auto genotype = initial_guess;
        if (i > 0) {
            for (auto& val : genotype) {
                val += dis(gen);
            }
        }
        population.emplace_back(genotype);
    }

    // Initialize fitness values.
    int iter = 0;
    auto fitnesses = std::vector<double>(population.size(), 0.0);

    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::system_clock::now() < timeout_point) {
        // Evaluate fitnesses
        std::cout << "Iteration " << iter << " Fitnesses: " << std::endl;
        for (size_t i = 0; i < population_size; ++i) {

            // Local gradient descent
            auto local_ik = GradientIk::from(population[i], cost_fn);
            auto constexpr timeout_local = 0.1;
            auto const timeout_point_local =
                std::chrono::system_clock::now() + std::chrono::duration<double>(timeout_local);
            while (std::chrono::system_clock::now() < timeout_point_local) {
                step(local_ik, robot, cost_fn);
            }    
            population[i] = local_ik.best;

            fitnesses[i] = cost_fn(population[i]);
            std::cout << i << ": " << fitnesses[i] << " " << std::endl;
        }
        std::cout << std::endl;

        // Sort fitnesses
        for (size_t i = 0; i < population_size; ++i) {
            if (solution_fn(population[i])) {
                return population[i];
            }
        }

        // Perturb if no solution found
        for (auto& genotype : population) {
            for (auto& val : genotype) {
                val += dis(gen);
            }
        }

        iter++;
    }

    if (approx_solution) {
        return population[0];
    }

    return std::nullopt;
}

}  // namespace pick_ik
