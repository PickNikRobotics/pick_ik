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
    auto constexpr max_iters_local = 25;
    auto const timeout_point_local =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout_local);

    int iter = 0;
    while (std::chrono::system_clock::now() < timeout_point_local && iter < max_iters_local) {
        step(local_ik, robot, cost_fn);
        iter++;
    }

    population_[i].genes = local_ik.best;
    population_[i].fitness = cost_fn(population_[i].genes);
}

void MemeticIk::initPopulation(Robot const& robot,
                               CostFn const& cost_fn,
                               std::vector<double> const& initial_guess) {
    population_.resize(population_count_);
    for (size_t i = 0; i < elite_count_; ++i) {
        auto genotype = initial_guess;
        if (i > 0) {
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto const& var = robot.variables[j_idx];
                genotype[j_idx] = mix_dist_(gen_) * var.span + var.min;
            }
        }
        population_[i] = Individual{genotype, cost_fn(genotype)};
    }

    // Initialize children to some dummy values that will be overwritten.
    for (size_t i = elite_count_; i < population_count_; ++i) {
        population_[i] = Individual{initial_guess, 0.0};
    }
}

void MemeticIk::reproduce(Robot const& robot, CostFn const& cost_fn) {
    std::vector<Individual*> pool;
    pool.reserve(elite_count_);
    for (size_t i = 0; i < elite_count_; ++i) {
        pool.emplace_back(&population_[i]);
    }

    for (size_t i = elite_count_; i < population_count_; ++i) {
        // Select parents
        // TODO: Make this selection better
        if (pool.size() > 1) {
            std::uniform_int_distribution<size_t> int_dist(0, pool.size());
            size_t const idxA = int_dist(gen_);
            size_t idxB = idxA;
            while (idxB == idxA && pool.size() > 1) {
                idxB = int_dist(gen_);
            }
            auto& parentA = population_[idxA];
            auto& parentB = population_[idxB];

            // Recombine and mutate
            auto const mix_ratio = mix_dist_(gen_);
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                population_[i].genes[j_idx] =
                    mix_ratio * parentA.genes[j_idx] + (1.0 - mix_ratio) * parentB.genes[j_idx];

                // Mutate (TODO with extinction factor)
                population_[i].genes[j_idx] += mutate_dist_(gen_);

                // Clamp to valid joint values
                population_[i].genes[j_idx] = std::clamp(population_[i].genes[j_idx],
                                                         robot.variables[j_idx].clip_min,
                                                         robot.variables[j_idx].clip_max);
            }

            // Evaluate fitness.
            population_[i].fitness = cost_fn(population_[i].genes);

            // Remove elements from the mating pool whose children have better fitness.
            if (population_[i].fitness < parentA.fitness)
                pool.erase(remove(pool.begin(), pool.end(), &parentA), pool.end());
            if (population_[i].fitness < parentB.fitness)
                pool.erase(remove(pool.begin(), pool.end(), &parentB), pool.end());

        } else {
            // Roll a new population member randomly.
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto const& var = robot.variables[j_idx];
                population_[i].genes[j_idx] = mix_dist_(gen_) * var.span + var.min;
            }
            population_[i].fitness = cost_fn(population_[i].genes);
        }
    }
}

void MemeticIk::printPopulation() const {
    std::cout << "Fitnesses: " << std::endl;
    for (size_t i = 0; i < populationCount(); ++i) {
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

    ik.initPopulation(robot, cost_fn, initial_guess);

    // Main loop
    int iter = 0;
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::system_clock::now() < timeout_point) {
        // Do gradient descent on elites.
        for (size_t i = 0; i < ik.eliteCount(); ++i) {
            ik.gradientDescent(i, robot, cost_fn);
        }

        // Perform mutation and recombination
        ik.reproduce(robot, cost_fn);

        // Sort fitnesses
        ik.sortPopulation();

        // Debug print
        std::cout << "Iteration " << iter << " ";
        ik.printPopulation();

        if (solution_fn(ik.best())) {
            std::cout << "Found solution!" << std::endl;
            return ik.best();
        }

        iter++;
    }

    if (approx_solution) {
        std::cout << "Returning best solution." << std::endl;
        return ik.best();
    }

    return std::nullopt;
}

}  // namespace pick_ik
