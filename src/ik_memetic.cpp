#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <vector>

namespace pick_ik {

MemeticIk MemeticIk::from(std::vector<double> const& initial_guess, CostFn const& cost_fn) {
    return MemeticIk{std::vector<double>(initial_guess.size(), 0.0), cost_fn(initial_guess)};
}

void MemeticIk::gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn) {
    auto local_ik = GradientIk::from(population_[i].genes, cost_fn);
    auto constexpr timeout_local = 0.01;
    auto const timeout_point_local =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout_local);
    while (std::chrono::system_clock::now() < timeout_point_local) {
        step(local_ik, robot, cost_fn);
    }

    population_[i].genes = local_ik.best;
    population_[i].fitness = cost_fn(population_[i].genes);
}

void MemeticIk::initPopulation(size_t const& population_size,
                               Robot const& robot,
                               CostFn const& cost_fn,
                               std::vector<double> const& initial_guess) {
    population_.reserve(population_size);
    for (size_t i = 0; i < population_size; ++i) {
        auto genotype = initial_guess;
        if (i > 0) {
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                genotype[j_idx] = std::clamp(genotype[j_idx] + dist_(gen_),
                                             robot.variables[j_idx].clip_min,
                                             robot.variables[j_idx].clip_max);
            }
        }
        population_.emplace_back(Individual{genotype, cost_fn(genotype)});
    }
}

size_t MemeticIk::populationSize() const { return population_.size(); }

void MemeticIk::printPopulation() const {
    std::cout << "Fitnesses: " << std::endl;
    for (size_t i = 0; i < populationSize(); ++i) {
        std::cout << i << ": " << population_[i].fitness << " " << std::endl;
    }
    std::cout << std::endl;
}

void MemeticIk::sortPopulation() {
    std::sort(population_.begin(), population_.end(), [](Individual const& a, Individual const& b) {
        return a.fitness < b.fitness;
    });
    best_ = population_[0].genes;
    cost_ = population_[0].fitness;
}

void MemeticIk::selectPopulation(Robot const& robot, CostFn const& cost_fn) {
    for (size_t i = 0; i < populationSize(); ++i) {
        if (i >= elite_count_) {
            population_[i] =
                population_[i - elite_count_];  // Bad selection criteria, should sample
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                population_[i].genes[j_idx] = std::clamp(population_[i].genes[j_idx] + dist_(gen_),
                                                         robot.variables[j_idx].clip_min,
                                                         robot.variables[j_idx].clip_max);
            }
        }
        population_[i].fitness = cost_fn(population_[i].genes);
    }
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

    size_t constexpr pop_size = 16;
    ik.initPopulation(pop_size, robot, cost_fn, initial_guess);

    // Initialize fitness values.
    int iter = 0;
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::system_clock::now() < timeout_point) {
        // Do gradient descent
        // TODO: Better selection of which ones to do gradient descent on.
        for (size_t i = 0; i < ik.populationSize(); ++i) {
            ik.gradientDescent(i, robot, cost_fn);
        }

        // Sort fitnesses
        ik.sortPopulation();

        // Debug print
        std::cout << "Iteration " << iter << " ";
        ik.printPopulation();

        if (solution_fn(ik.best())) {
            std::cout << "Found solution!" << std::endl;
            return ik.best();
        }

        ik.selectPopulation(robot, cost_fn);
        iter++;
    }

    if (approx_solution) {
        std::cout << "Returning best solution." << std::endl;
        //     return population_[0];
    }

    return std::nullopt;
}

}  // namespace pick_ik
