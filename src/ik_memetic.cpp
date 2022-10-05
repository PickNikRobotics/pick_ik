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

MemeticIk MemeticIk::from(std::vector<double> const& initial_guess,
                          CostFn const& cost_fn,
                          MemeticIkParams const& params) {
    return MemeticIk{initial_guess, cost_fn(initial_guess), params};
}

MemeticIk::MemeticIk(std::vector<double> const& initial_guess,
                     double cost,
                     MemeticIkParams const& params)
    : params_{params} {
    best_ = Individual{initial_guess, cost, 0.0, std::vector<double>(initial_guess.size(), 0.0)};
    best_curr_ = best_;

    // Cache extinction grading coefficients to not have to recompute all the time.
    extinction_grading_.reserve(params.population_size);
    for (size_t i = 0; i < params.population_size; ++i) {
        extinction_grading_.push_back(static_cast<double>(i) /
                                      static_cast<double>(params.population_size - 1));
    }
};

void MemeticIk::computeExtinctions() {
    double min_fitness = population_.front().fitness;
    double max_fitness = population_.back().fitness;
    for (size_t i = 0; i < params_.population_size; ++i) {
        population_[i].extinction =
            (population_[i].fitness + min_fitness * (extinction_grading_[i] - 1)) / max_fitness;
    }
}

void MemeticIk::gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn) {
    auto& individual = population_[i];
    auto local_ik = GradientIk::from(individual.genes, cost_fn);
    auto const timeout_point_local =
        std::chrono::system_clock::now() + std::chrono::duration<double>(params_.local_max_time);

    int iter = 0;
    while (std::chrono::system_clock::now() < timeout_point_local &&
           iter < params_.local_max_iters) {
        if (!step(local_ik, robot, cost_fn, params_.local_step_size)) {
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
    population_.resize(params_.population_size);
    for (size_t i = 0; i < params_.elite_size; ++i) {
        auto genotype = initial_guess;
        if (i > 0) {
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto const& var = robot.variables[j_idx];
                genotype[j_idx] = rsl::uniform_real(0.0, 1.0) * var.span + var.min;
            }
        }
        population_[i] = Individual{genotype, cost_fn(genotype), 1.0, zero_grad};
    }

    // Initialize children to some dummy values that will be overwritten.
    for (size_t i = params_.elite_size; i < params_.population_size; ++i) {
        population_[i] = Individual{initial_guess, 0.0, 1.0, zero_grad};
    }

    computeExtinctions();
}

void MemeticIk::reproduce(Robot const& robot, CostFn const& cost_fn) {
    std::vector<Individual*> pool;
    pool.reserve(params_.elite_size);
    for (size_t i = 0; i < params_.elite_size; ++i) {
        pool.emplace_back(&population_[i]);
    }

    for (size_t i = params_.elite_size; i < params_.population_size; ++i) {
        // Select parents
        // TODO: Make this code better
        if (pool.size() > 1) {
            size_t const idxA = rsl::uniform_int<size_t>(0, pool.size());
            size_t idxB = idxA;
            while (idxB == idxA && pool.size() > 1) {
                idxB = rsl::uniform_int<size_t>(0, pool.size());
            }
            auto& parentA = population_[idxA];
            auto& parentB = population_[idxB];

            // Get mutation probability
            double const extinction = 0.5 * (parentA.extinction + parentB.extinction);
            double const inverse = 1.0 / static_cast<double>(robot.variables.size());
            double const mutation_prob = extinction * (1.0 - inverse) + inverse;

            auto const mix_ratio = rsl::uniform_real(0.0, 1.0);
            for (size_t j_idx = 0; j_idx < robot.variables.size(); ++j_idx) {
                auto& gene = population_[i].genes[j_idx];
                auto joint = robot.variables[j_idx];

                // Reproduce
                gene = mix_ratio * parentA.genes[j_idx] + (1.0 - mix_ratio) * parentB.genes[j_idx];

                // Add in parent gradients
                gene += rsl::uniform_real(0.0, 1.0) * parentA.gradient[j_idx] +
                        rsl::uniform_real(0.0, 1.0) * parentB.gradient[j_idx];
                auto original_gene = gene;

                // Mutate
                if (rsl::uniform_real(0.0, 1.0) < mutation_prob) {
                    gene += extinction * joint.span * rsl::uniform_real(-0.5, 0.5);
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
                population_[i].genes[j_idx] = rsl::uniform_real(0.0, 1.0) * var.span + var.min;
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
    best_curr_ = population_[0];
    if (best_curr_.fitness < best_.fitness) {
        best_ = best_curr_;
    }
}

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                MemeticIkParams const& params,
                double const timeout,
                bool const approx_solution,
                bool const print_debug) -> std::optional<std::vector<double>> {
    if (solution_fn(initial_guess)) {
        return initial_guess;
    }

    assert(robot.variables.size() == initial_guess.size());
    auto ik = MemeticIk::from(initial_guess, cost_fn, params);

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
        if (solution_fn(ik.best().genes)) {
            if (print_debug) fmt::print("Found solution!\n");
            return ik.best().genes;
        }

        // Handle wipeouts if no progress is being made.
        if (previous_fitness.has_value()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            bool const improved =
                (ik.bestCurrent().fitness < *previous_fitness - params.wipeout_fitness_tol);
#pragma GCC diagnostic pop
            if (!improved) {
                if (print_debug) fmt::print("Population wipeout\n");
                ik.initPopulation(robot, cost_fn, initial_guess);
                previous_fitness.reset();
            } else {
                previous_fitness = ik.bestCurrent().fitness;
            }
        } else {
            previous_fitness = ik.bestCurrent().fitness;
        }
        iter++;
    }

    if (approx_solution) {
        if (print_debug) fmt::print("Returning best solution\n");
        return ik.best().genes;
    }

    return std::nullopt;
}

}  // namespace pick_ik
