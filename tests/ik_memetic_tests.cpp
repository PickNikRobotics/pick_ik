#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/utils/robot_model_test_utils.h>

// Helper param struct and function to test IK solution.
struct MemeticIkTestParams {
    double twist_threshold = 0.0001;
    double cost_threshold = 0.0001;
    double rotation_scale = 1.0;
    double timeout = 1.0;
    bool return_approximate_solution = false;
};

auto solve_memetic_ik_test(moveit::core::RobotModelPtr robot_model,
                           std::string const group_name,
                           std::string const goal_frame_name,
                           Eigen::Isometry3d const& goal_frame,
                           std::vector<double> const& initial_guess,
                           MemeticIkTestParams const& params = MemeticIkTestParams())
    -> std::optional<std::vector<double>> {
    // Make forward kinematics function
    auto const jmg = robot_model->getJointModelGroup(group_name);
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {goal_frame_name}).value();
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, tip_link_indices);

    // Make solution function
    auto const test_rotation = (params.rotation_scale > 0.0);
    auto const frame_tests =
        pick_ik::make_frame_tests({goal_frame}, params.twist_threshold, test_rotation);
    auto const cost_function =
        kinematics::KinematicsBase::IKCostFn();  // What should be instantiated here?
    std::vector<pick_ik::Goal> goals = {};       // TODO: Only works if empty.
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk_fn);

    // Make pose cost function
    auto const pose_cost_functions =
        pick_ik::make_pose_cost_functions({goal_frame}, params.rotation_scale);
    CHECK(pose_cost_functions.size() == 1);

    // Solve IK
    auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);
    auto const cost_fn = pick_ik::make_cost_fn(pose_cost_functions, goals, fk_fn);
    return pick_ik::ik_memetic(initial_guess, robot, cost_fn, solution_fn, params.timeout, true);
}

TEST_CASE("Panda model IK") {
    using moveit::core::loadTestingRobotModel;
    auto const robot_model = loadTestingRobotModel("panda");

    auto const jmg = robot_model->getJointModelGroup("panda_arm");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"panda_hand"}).value();
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, tip_link_indices);

    std::vector<double> const home_joint_angles =
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};

    // SECTION("Panda model IK at home positions.") {
    //     auto const goal_frame = fk_fn(home_joint_angles)[0];
    //     auto initial_guess = home_joint_angles;
    //     MemeticIkTestParams params;
    //     params.cost_threshold = 0.0001;
    //     params.twist_threshold = 0.0001;
    //     params.timeout = 10.0;

    //     auto const maybe_solution = solve_memetic_ik_test(robot_model,
    //                                                       "panda_arm",
    //                                                       "panda_hand",
    //                                                       goal_frame,
    //                                                       initial_guess,
    //                                                       params);

    //     REQUIRE(maybe_solution.has_value());
    //     auto const final_frame = fk_fn(maybe_solution.value())[0];
    //     CHECK(goal_frame.isApprox(final_frame, params.twist_threshold));
    // }

    // SECTION("Panda model IK near home positions.") {
    //     auto const goal_frame = fk_fn(home_joint_angles)[0];
    //     auto initial_guess = home_joint_angles;
    //     std::vector<double> const initial_guess_offsets = {0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1};
    //     for (size_t i=0; i < initial_guess.size(); ++i) {
    //         initial_guess[i] += initial_guess_offsets[i];
    //     }
    //     MemeticIkTestParams params;
    //     params.cost_threshold = 0.0001;
    //     params.twist_threshold = 0.0001;
    //     params.timeout = 10.0;

    //     auto const maybe_solution = solve_memetic_ik_test(robot_model,
    //                                                       "panda_arm",
    //                                                       "panda_hand",
    //                                                       goal_frame,
    //                                                       initial_guess,
    //                                                       params);

    //     REQUIRE(maybe_solution.has_value());
    //     auto const final_frame = fk_fn(maybe_solution.value())[0];
    //     CHECK(goal_frame.isApprox(final_frame, params.twist_threshold));
    // }

    SECTION("Panda model IK at zero positions.") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto const initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        MemeticIkTestParams params;
        params.cost_threshold = 0.0001;
        params.twist_threshold = 0.0001;
        params.timeout = 15.0;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.twist_threshold));
    }
}
