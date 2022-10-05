#pragma once

#include <pick_ik/goal.hpp>
#include <pick_ik/robot.hpp>

#include <chrono>
#include <memory>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <optional>
#include <random>
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

    // Gradient descent parameters
    double local_step_size = 0.0001;  // Joint angle numerical perturbation step size.
    int local_max_iters = 25;         // Max iterations per memetic exploitation step.
    double local_max_time = 0.005;    // Max wall time per memetic exploitation step, in seconds.
};

class MemeticIk {
   private:
    std::vector<Individual> population_;
    Individual best_;       // Best solution overall.
    Individual best_curr_;  // Best solution so far.

    MemeticIkParams params_;

    // Extinction coefficient gradings (cached since they do not change).
    std::vector<double> extinction_grading_;

    // RNG (TODO use RSL)
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> mutate_dist_{-0.5, 0.5};
    std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};

   public:
    MemeticIk(std::vector<double> const& initial_guess, double cost, MemeticIkParams const& params);
    static MemeticIk from(std::vector<double> const& initial_guess,
                          CostFn const& cost_fn,
                          MemeticIkParams const& params);

    Individual best() const { return best_; };
    Individual bestCurrent() const { return best_curr_; };
    size_t eliteCount() const { return params_.elite_size; };
    void computeExtinctions();
    void gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn);
    void initPopulation(Robot const& robot,
                        CostFn const& cost_fn,
                        std::vector<double> const& initial_guess);
    void reproduce(Robot const& robot, CostFn const& cost_fn);
    size_t populationCount() const { return params_.population_size; };
    void printPopulation() const;
    void sortPopulation();
};

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                MemeticIkParams const& params,
                double const timeout = 1.0,
                bool const approx_solution = false,
                bool const print_debug = false) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
