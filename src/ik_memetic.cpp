#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <optional>
#include <vector>

namespace pick_ik {

MemeticIk MemeticIk::from(std::vector<double> const& initial_guess, CostFn const& cost_fn) {
    return MemeticIk{std::vector<double>(initial_guess.size(), 0.0), cost_fn(initial_guess)};
}

void MemeticIk::computeExtinctions() {
    double min_fitness = population_.front().fitness;
    double max_fitness = population_.back().fitness;
    for (size_t i = 0; i < population_count_; ++i) {
        double grading = static_cast<double>(i) / static_cast<double>(population_count_ - 1);
        population_[i].extinction =
            (population_[i].fitness + min_fitness * (grading - 1)) / max_fitness;
    }
}

void MemeticIk::gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn) {
    auto& individual = population_[i];
    auto local_ik = GradientIk::from(individual.genes, cost_fn);
    auto constexpr timeout_local = 0.005;
    auto constexpr max_iters_local = 25;
    auto const timeout_point_local =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout_local);

    int iter = 0;
    while (std::chrono::system_clock::now() < timeout_point_local && iter < max_iters_local) {
        if (!step(local_ik, robot, cost_fn)) {
            break;
        }
        iter++;
    }

    individual.genes = local_ik.best;
    individual.fitness = cost_fn(individual.genes);
    individual.gradient = local_ik.gradient;
}

void MemeticIk::initPopulation(Robot const& robot,
                               CostFn const& cost_fn,
                               std::vector<double> const& initial_guess) {
    std::vector<double> const zero_grad(robot.variables.size(), 0.0);
    population_.resize(population_count_);
    for (size_t i = 0; i < elite_count_; ++i) {
        auto genotype = initial_guess;
        if (i > 0) {
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto const& var = robot.variables[j_idx];
                genotype[j_idx] = uniform_dist_(gen_) * var.span + var.min;
            }
        }
        population_[i] = Individual{genotype, cost_fn(genotype), 1.0, zero_grad};
    }

    // Initialize children to some dummy values that will be overwritten.
    for (size_t i = elite_count_; i < population_count_; ++i) {
        population_[i] = Individual{initial_guess, 0.0, 1.0, zero_grad};
    }

    computeExtinctions();
}

void MemeticIk::reproduce(Robot const& robot, CostFn const& cost_fn) {
    std::vector<Individual*> pool;
    pool.reserve(elite_count_);
    for (size_t i = 0; i < elite_count_; ++i) {
        pool.emplace_back(&population_[i]);
    }

    for (size_t i = elite_count_; i < population_count_; ++i) {
        // Select parents
        // TODO: Make this code better
        if (pool.size() > 1) {
            std::uniform_int_distribution<size_t> int_dist(0, pool.size());
            size_t const idxA = int_dist(gen_);
            size_t idxB = idxA;
            while (idxB == idxA && pool.size() > 1) {
                idxB = int_dist(gen_);
            }
            auto& parentA = population_[idxA];
            auto& parentB = population_[idxB];

            // Get mutation probability
            double const extinction = 0.5 * (parentA.extinction + parentB.extinction);
            double const inverse = 1.0 / static_cast<double>(robot.variables.size());
            double const mutation_prob = extinction * (1.0 - inverse) + inverse;

            auto const mix_ratio = uniform_dist_(gen_);
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto& gene = population_[i].genes[j_idx];
                auto joint = robot.variables[j_idx];

                // Reproduce
                gene = mix_ratio * parentA.genes[j_idx] + (1.0 - mix_ratio) * parentB.genes[j_idx];

                // Add in parent gradients
                gene += uniform_dist_(gen_) * parentA.gradient[j_idx] +
                        uniform_dist_(gen_) * parentB.gradient[j_idx];
                auto original_gene = gene;

                // Mutate
                if (uniform_dist_(gen_) < mutation_prob) {
                    gene += extinction * joint.span * mutate_dist_(gen_);
                }

                // Clamp to valid joint values
                gene = std::clamp(gene, joint.clip_min, joint.clip_max);

                // Approximate initial gradient
                population_[i].gradient[j_idx] = gene - original_gene;
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
                population_[i].genes[j_idx] = uniform_dist_(gen_) * var.span + var.min;
            }
            population_[i].fitness = cost_fn(population_[i].genes);
            for (auto& g : population_[i].gradient) {
                g = 0.0;
            }
        }
    }
}

void MemeticIk::printPopulation() const {
    fmt::print("Fitnesses:\n");
    for (size_t i = 0; i < populationCount(); ++i) {
        fmt::print("{}: {:.4f}\n", i, population_[i].fitness);
    }
    fmt::print("\n");
}

void MemeticIk::sortPopulation() {
    std::sort(population_.begin(), population_.end(), [](Individual const& a, Individual const& b) {
        return a.fitness < b.fitness;
    });
    best_curr_ = population_[0].genes;
    best_cost_curr_ = population_[0].fitness;
    if (best_cost_curr_ < best_cost_) {
        best_ = best_curr_;
        best_cost_ = best_cost_curr_;
    }
}

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                double const timeout,
                bool const approx_solution,
                bool const print_debug) -> std::optional<std::vector<double>> {
    if (solution_fn(initial_guess)) {
        return initial_guess;
    }

    assert(robot.variables.size() == initial_guess.size());
    auto ik = MemeticIk::from(initial_guess, cost_fn);

    ik.initPopulation(robot, cost_fn, initial_guess);

    // Main loop
    int iter = 0;
    std::optional<double> previous_fitness = std::nullopt;
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
        ik.computeExtinctions();
        if (print_debug) {
            fmt::print("Iteration {}\n", iter);
            ik.printPopulation();
        }

        // Check for termination
        if (solution_fn(ik.best())) {
            if (print_debug) fmt::print("Found solution!\n");
            return ik.best();
        }

        // Handle wipeouts if no progress is being made.
        auto constexpr improve_tol = 0.00001;  // TODO Promote
        if (previous_fitness.has_value()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            bool const improved = (ik.bestCurrentCost() < *previous_fitness - improve_tol);
#pragma GCC diagnostic pop
            if (!improved) {
                if (print_debug) fmt::print("Population wipeout");
                ik.initPopulation(robot, cost_fn, initial_guess);
                previous_fitness.reset();
            } else {
                previous_fitness = ik.bestCurrentCost();
            }
        } else {
            previous_fitness = ik.bestCurrentCost();
        }
        iter++;
    }

    if (approx_solution) {
        if (print_debug) fmt::print("Returning best solution\n");
        return ik.best();
    }

    return std::nullopt;
}

}  // namespace pick_ik
