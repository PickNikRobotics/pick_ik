#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <rsl/queue.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fmt/core.h>
#include <limits>
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
        bool const improved =
            (best_curr_.fitness < previous_fitness_.value() - params_.wipeout_fitness_tol);
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

void MemeticIk::gradientDescent(size_t const i,
                                Robot const& robot,
                                CostFn const& cost_fn,
                                GradientIkParams const& gd_params) {
    auto& individual = population_[i];
    auto local_ik = GradientIk::from(individual.genes, cost_fn);

    int num_iterations = 0;
    double previous_cost = 0;
    auto const timeout_point_local =
        std::chrono::system_clock::now() + std::chrono::duration<double>(gd_params.max_time);

    while ((std::chrono::system_clock::now() < timeout_point_local) &&
           (num_iterations < gd_params.max_iterations)) {
        step(local_ik, robot, cost_fn, gd_params.step_size);
        if (abs(local_ik.local_cost - previous_cost) <= gd_params.min_cost_delta) {
            break;
        }
        previous_cost = local_ik.local_cost;
        num_iterations++;
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
            robot.set_random_valid_configuration(genotype);
        }
        population_[i] = Individual{genotype, cost_fn(genotype), 1.0, zero_grad};
    }

    // Initialize children to some dummy values that will be overwritten.
    for (size_t i = params_.elite_size; i < params_.population_size; ++i) {
        population_[i] = Individual{initial_guess, 0.0, 1.0, zero_grad};
    }

    // Initialize fitnesses and extinctions
    for (auto& individual : population_) {
        individual.fitness = cost_fn(individual.genes);
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
            size_t const idxA = rsl::uniform_int<size_t>(0, mating_pool_.size() - 1);
            size_t idxB = idxA;
            while (idxB == idxA && mating_pool_.size() > 1) {
                idxB = rsl::uniform_int<size_t>(0, mating_pool_.size() - 1);
            }
            auto& parentA = *(mating_pool_[idxA]);
            auto& parentB = *(mating_pool_[idxB]);

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
                    gene += extinction * joint.half_span * rsl::uniform_real(-1.0, 1.0);
                }

                // Clamp to valid joint values
                gene = robot.variables[j_idx].clamp_to_limits(gene);

                // Approximate gradient
                population_[i].gradient[j_idx] = gene - original_gene;
            }

            // Evaluate fitness and remove parents from the mating pool if a child with better
            // fitness exists.
            population_[i].fitness = cost_fn(population_[i].genes);
            if (population_[i].fitness < parentA.fitness) {
                auto it = std::find(mating_pool_.begin(), mating_pool_.end(), &parentA);
                if (it != mating_pool_.end()) mating_pool_.erase(it);
            }
            if (population_[i].fitness < parentB.fitness) {
                auto it = std::find(mating_pool_.begin(), mating_pool_.end(), &parentB);
                if (it != mating_pool_.end()) mating_pool_.erase(it);
            }

        } else {
            // If the mating pool is empty, roll a new population member randomly.
            robot.set_random_valid_configuration(population_[i].genes);
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

auto ik_memetic_impl(std::vector<double> const& initial_guess,
                     Robot const& robot,
                     CostFn const& cost_fn,
                     SolutionTestFn const& solution_fn,
                     MemeticIkParams const& params,
                     std::atomic<bool>& terminate,
                     bool approx_solution,
                     bool print_debug) -> std::optional<Individual> {
    assert(robot.variables.size() == initial_guess.size());
    auto ik = MemeticIk::from(initial_guess, cost_fn, params);

    ik.initPopulation(robot, cost_fn, initial_guess);

    // Main loop
    int iter = 0;
    auto const timeout_point =
        std::chrono::system_clock::now() + std::chrono::duration<double>(params.max_time);
    while ((std::chrono::system_clock::now() < timeout_point) && (iter < params.max_generations)) {
        // Do gradient descent on elites.
        std::vector<std::thread> gd_threads;
        gd_threads.reserve(ik.eliteCount());
        for (size_t i = 0; i < ik.eliteCount(); ++i) {
            gd_threads.push_back(std::thread([&ik, i, &robot, cost_fn, &params] {
                ik.gradientDescent(i, robot, cost_fn, params.gd_params);
            }));
        }
        for (auto& t : gd_threads) {
            t.join();
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
        if (params.stop_optimization_on_valid_solution && solution_fn(ik.best().genes)) {
            if (print_debug) fmt::print("Found solution!\n");
            return ik.best();
        }
        if (ik.checkWipeout()) {
            if (print_debug) fmt::print("Population wipeout\n");
            ik.initPopulation(robot, cost_fn, initial_guess);
        }

        // Check termination condition from other threads finding a solution.
        if (terminate) {
            if (print_debug) fmt::print("Terminated\n");
            break;
        }

        iter++;
    }

    // If we kept optimizing, we need to check if we found a valid solution
    if (!params.stop_optimization_on_valid_solution && solution_fn(ik.best().genes)) {
        if (print_debug) fmt::print("Found solution!\n");
        return ik.best();
    }

    if (approx_solution) {
        if (print_debug) fmt::print("Returning best solution\n");
        return ik.best();
    }

    return std::nullopt;
}

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                MemeticIkParams const& params,
                bool approx_solution,
                bool print_debug) -> std::optional<std::vector<double>> {
    // Check whether the initial guess already meets the goal,
    // before starting to solve.
    if (params.stop_optimization_on_valid_solution && solution_fn(initial_guess)) {
        return initial_guess;
    }

    std::atomic<bool> terminate{false};
    if (params.num_threads <= 1) {
        // Single-threaded implementation
        auto maybe_solution = ik_memetic_impl(initial_guess,
                                              robot,
                                              cost_fn,
                                              solution_fn,
                                              params,
                                              terminate,
                                              approx_solution,
                                              print_debug);
        if (maybe_solution.has_value()) {
            return maybe_solution.value().genes;
        }
    } else {
        // Multi-threaded implementation
        rsl::Queue<std::optional<Individual>> solution_queue;
        std::vector<std::thread> ik_threads;
        ik_threads.reserve(params.num_threads);

        auto ik_thread_fn = [=, &terminate, &solution_queue]() {
            auto soln = ik_memetic_impl(initial_guess,
                                        robot,
                                        cost_fn,
                                        solution_fn,
                                        params,
                                        terminate,
                                        approx_solution,
                                        print_debug);
            solution_queue.push(soln);
        };

        for (size_t i = 0; i < params.num_threads; ++i) {
            ik_threads.push_back(std::thread(ik_thread_fn));
        }

        // If enabled, stop all other threads once one thread finds a valid solution.
        size_t n_threads_done = 0;
        std::vector<double> best_solution;
        auto min_cost = std::numeric_limits<double>::max();
        auto maybe_solution = std::optional<std::optional<Individual>>{std::nullopt};
        if (params.stop_on_first_soln) {
            while (!maybe_solution && (n_threads_done < params.num_threads)) {
                maybe_solution = solution_queue.pop(std::chrono::milliseconds(1));
            }
            if (maybe_solution.value().has_value()) {
                auto const& solution = maybe_solution.value().value();
                best_solution = solution.genes;
                min_cost = solution.fitness;
                terminate = true;
            }
            n_threads_done++;
        }

        for (auto& t : ik_threads) {
            t.join();
        }

        // Get the minimum-cost solution from all threads.
        // Note that if approximate solutions are enabled, even if we terminate threads early, we
        // can still compare our first solution with the approximate ones from the other threads
        while (!solution_queue.empty()) {
            maybe_solution = solution_queue.pop();
            if (maybe_solution.value().has_value()) {
                auto const& solution = maybe_solution.value().value();
                auto const& cost = solution.fitness;
                if (cost < min_cost) {
                    best_solution = solution.genes;
                    min_cost = cost;
                }
            }
        }
        if (!best_solution.empty()) return best_solution;
    }
    return std::nullopt;
}

}  // namespace pick_ik
