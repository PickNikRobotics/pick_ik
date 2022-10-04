#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <rsl/queue.hpp>
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

    population_.reserve(params.population_size);
    mating_pool_.reserve(params.elite_size);

    // Cache some coefficients to not have to recompute them all the time.
    extinction_grading_.reserve(params.population_size);
    for (size_t i = 0; i < params.population_size; ++i) {
        extinction_grading_.push_back(static_cast<double>(i) /
                                      static_cast<double>(params.population_size - 1));
    }
    inverse_gene_size_ = 1.0 / static_cast<double>(initial_guess.size());
};

bool MemeticIk::checkWipeout() {
    // Handle wipeouts if no progress is being made.
    if (previous_fitness_.has_value()) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        bool const improved =
            (best_curr_.fitness < *previous_fitness_ - params_.wipeout_fitness_tol);
#pragma GCC diagnostic pop
        if (!improved) {
            return true;
        }
    }

    previous_fitness_ = best_curr_.fitness;
    return false;
}

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
    previous_fitness_.reset();
}

void MemeticIk::reproduce(Robot const& robot, CostFn const& cost_fn) {
    // Reset mating pool
    mating_pool_.resize(params_.elite_size);
    for (size_t i = 0; i < params_.elite_size; ++i) {
        mating_pool_[i] = &population_[i];
    }

    for (size_t i = params_.elite_size; i < params_.population_size; ++i) {
        // Select parents from the mating pool
        // Note that we permit there being only one parent, which basically counts as just
        // mutations.
        if (!mating_pool_.empty()) {
            size_t const idxA = rsl::uniform_int<size_t>(0, mating_pool_.size());
            size_t idxB = idxA;
            while (idxB == idxA && mating_pool_.size() > 1) {
                idxB = rsl::uniform_int<size_t>(0, mating_pool_.size());
            }
            auto& parentA = population_[idxA];
            auto& parentB = population_[idxB];

            // Get mutation probability
            double const extinction = 0.5 * (parentA.extinction + parentB.extinction);
            double const mutation_prob =
                extinction * (1.0 - inverse_gene_size_) + inverse_gene_size_;

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

                // Approximate gradient
                population_[i].gradient[j_idx] = gene - original_gene;
            }

            // Evaluate fitness and remove parents from the mating pool if a child with better
            // fitness exists.
            population_[i].fitness = cost_fn(population_[i].genes);
            if (population_[i].fitness < parentA.fitness)
                mating_pool_.erase(remove(mating_pool_.begin(), mating_pool_.end(), &parentA),
                                   mating_pool_.end());
            if (population_[i].fitness < parentB.fitness)
                mating_pool_.erase(remove(mating_pool_.begin(), mating_pool_.end(), &parentB),
                                   mating_pool_.end());

        } else {
            // If the mating pool is empty, roll a new population member randomly.
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
        fmt::print("{}: {}\n", i, population_[i].fitness);
    }
    fmt::print("\n");
}

void MemeticIk::sortPopulation() {
    std::sort(population_.begin(), population_.end(), [](Individual const& a, Individual const& b) {
        return a.fitness < b.fitness;
    });
    computeExtinctions();
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
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(timeout);
    while (std::chrono::system_clock::now() < timeout_point) {
        // Do gradient descent on elites.
        for (size_t i = 0; i < ik.eliteCount(); ++i) {
            ik.gradientDescent(i, robot, cost_fn);
        }

        // Perform mutation and recombination
        ik.reproduce(robot, cost_fn);

        // Sort fitnesses and update extinctions
        ik.sortPopulation();
        if (print_debug) {
            fmt::print("Iteration {}\n", iter);
            ik.printPopulation();
        }

        // Check for termination and wipeout conditions
        if (solution_fn(ik.best().genes)) {
            if (print_debug) fmt::print("Found solution!\n");
            return ik.best().genes;
        }
        if (ik.checkWipeout()) {
            if (print_debug) fmt::print("Population wipeout\n");
            ik.initPopulation(robot, cost_fn, initial_guess);
        }

        iter++;
    }

    if (approx_solution) {
        if (print_debug) fmt::print("Returning best solution\n");
        return ik.best().genes;
    }

    return std::nullopt;
}

auto ik_memetic_multithreaded(std::vector<double> const& initial_guess,
                              Robot const& robot,
                              CostFn const& cost_fn,
                              SolutionTestFn const& solution_fn,
                              MemeticIkParams const& params,
                              size_t const num_threads,
                              double const timeout,
                              bool const approx_solution,
                              bool const print_debug) -> std::optional<std::vector<double>> {
    
    rsl::Queue<std::optional<std::vector<double>>> solution_queue;
    std::vector<std::thread> ik_threads;
    ik_threads.reserve(num_threads);

    auto ik_thread_fn = [=, &solution_queue]() {
        auto soln = ik_memetic(initial_guess,
                               robot,
                               cost_fn,
                               solution_fn,
                               params,
                               timeout,
                               approx_solution,
                               print_debug);
        solution_queue.push(soln);
    };

    for (size_t i = 0; i < num_threads; ++i) {
        ik_threads.push_back(std::thread(ik_thread_fn));
    }

    for (auto& t : ik_threads) {
        t.join();
    }

    // TODO: What if we just want the first solution and not the best?

    // TODO: Pick out solutions correctly, and not just the latest popped one.
    if (!solution_queue.empty()) {
        return solution_queue.pop().value();
    }

    return std::nullopt;
}

}  // namespace pick_ik
