#pragma once

#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/robot.hpp>

#include <rsl/random.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <optional>
#include <vector>

namespace pick_ik {

struct Individual {
    std::vector<double> genes;  // Joint angles
    double fitness;
    double extinction;
    std::vector<double> gradient;
};

struct MemeticIkParams {
    // Evolutionary algorithm parameters
    size_t elite_size = 4;                 // Number of "keep alive" population members.
    size_t population_size = 16;           // Number of total population members.
    double wipeout_fitness_tol = 0.00001;  // Min fitness must improve by at least this much or the
                                           // population is reinitialized.
    int max_generations = 100;             // Maximum iterations for evolutionary algorithm.
    double max_time = 1.0;                 // Maximum time for evolutionary algorithm.

    size_t num_threads = 1;  // Number of species to solve in parallel.
    // If true, returns first solution and terminates other threads.
    // If false, waits for all threads to join and returns best solution.
    bool stop_on_first_soln = true;

    // Gradient descent parameters for memetic exploitation.
    GradientIkParams gd_params;
};

class MemeticIk {
   private:
    // Evolutionary algorithm values
    std::vector<Individual> population_;
    std::vector<Individual*> mating_pool_;
    Individual best_;       // Best solution overall.
    Individual best_curr_;  // Best solution so far.
    std::optional<double> previous_fitness_;

    // Solver parameters
    MemeticIkParams params_;

    // Scaling coefficients (cached since they do not change).
    std::vector<double> extinction_grading_;
    double inverse_gene_size_;

   public:
    MemeticIk(std::vector<double> const& initial_guess, double cost, MemeticIkParams const& params);
    static MemeticIk from(std::vector<double> const& initial_guess,
                          CostFn const& cost_fn,
                          MemeticIkParams const& params);

    Individual best() const { return best_; };
    Individual bestCurrent() const { return best_curr_; };
    size_t eliteCount() const { return params_.elite_size; };
    bool checkWipeout();
    void computeExtinctions();
    void gradientDescent(size_t const i,
                         Robot const& robot,
                         CostFn const& cost_fn,
                         GradientIkParams const& gd_params);
    void initPopulation(Robot const& robot,
                        CostFn const& cost_fn,
                        std::vector<double> const& initial_guess);
    void reproduce(Robot const& robot, CostFn const& cost_fn);
    size_t populationCount() const { return params_.population_size; };
    void printPopulation() const;
    void sortPopulation();
};

// Implementation of memetic IK solve.
auto ik_memetic_impl(std::vector<double> const& initial_guess,
                     Robot const& robot,
                     CostFn const& cost_fn,
                     SolutionTestFn const& solution_fn,
                     MemeticIkParams const& params,
                     std::atomic<bool>& terminate,
                     bool approx_solution = false,
                     bool print_debug = false) -> std::optional<Individual>;

// Top-level IK solution implementation that handles single vs. multithreading.
auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                MemeticIkParams const& params,
                bool approx_solution = false,
                bool print_debug = false) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
