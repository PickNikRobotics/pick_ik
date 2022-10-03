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

    // Config params
    size_t elite_count_ = 2;

    // RNG (TODO move out of here)
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> dist_{-M_PI_4, M_PI_4};  // TODO Configure

   public:
    MemeticIk(std::vector<double> const& initial_guess, double cost)
        : best_{initial_guess}, cost_{cost}, gen_{rd_()} {};
    static MemeticIk from(std::vector<double> const& initial_guess, CostFn const& cost_fn);

    std::vector<double> best() { return best_; };
    void gradientDescent(size_t const i, Robot const& robot, CostFn const& cost_fn);
    void initPopulation(size_t const& population_size,
                        Robot const& robot,
                        CostFn const& cost_fn,
                        std::vector<double> const& initial_guess);
    size_t populationSize() const;
    void printPopulation() const;
    void sortPopulation();
    void selectPopulation(Robot const& robot, CostFn const& cost_fn);
};

// auto step(MemeticIk& self, Robot const& robot, CostFn const& cost_fn) -> bool;

auto ik_memetic(std::vector<double> const& initial_guess,
                Robot const& robot,
                CostFn const& cost_fn,
                SolutionTestFn const& solution_fn,
                double timeout = 10.0,
                bool approx_solution = false) -> std::optional<std::vector<double>>;

}  // namespace pick_ik
