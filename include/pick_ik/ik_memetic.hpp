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
};

class MemeticIk {
   private:
    std::vector<double> best_;
    double cost_;
    std::vector<Individual> population_;
    std::vector<Individual> children_;

    // Config params
    size_t elite_count_ = 4;
    size_t population_count_ = 16;

    // RNG (TODO move out of here)
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> mutate_dist_{-0.2, 0.2};
    std::uniform_real_distribution<double> mix_dist_{0.0, 1.0};

   public:
    MemeticIk(std::vector<double> const& initial_guess, double cost)
        : best_{initial_guess}, cost_{cost}, gen_{rd_()} {};
    static MemeticIk from(std::vector<double> const& initial_guess, CostFn const& cost_fn);

    std::vector<double> best() { return best_; };
    size_t eliteCount() const { return elite_count_; };
    void gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn);
    void initPopulation(Robot const& robot,
                        CostFn const& cost_fn,
                        std::vector<double> const& initial_guess);
    void reproduce(Robot const& robot, CostFn const& cost_fn);
    size_t populationCount() const { return population_count_; };
    void printPopulation() const;
    void sortPopulation();
};

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                double timeout = 10.0,
                bool approx_solution = false) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
