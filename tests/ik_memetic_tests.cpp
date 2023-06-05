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
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/utils/robot_model_test_utils.h>

// Helper param struct and function to test IK solution.
struct MemeticIkTestParams {
    double position_threshold = 0.001;
    double orientation_threshold = 0.01;
    double cost_threshold = 0.001;
    double position_scale = 1.0;
    double rotation_scale = 0.5;

    // Solve options
    bool approximate_solution = false;
    bool print_debug = false;
    pick_ik::MemeticIkParams memetic_params;

    // Additional costs
    double center_joints_weight = 0.0;
    double avoid_joint_limits_weight = 0.0;
    double minimal_displacement_weight = 0.0;
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
    std::mutex mx;
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, mx, tip_link_indices);
    auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);

    // Make goal function(s)
    std::vector<pick_ik::Goal> goals = {};
    if (params.center_joints_weight > 0) {
        goals.push_back(
            pick_ik::Goal{pick_ik::make_center_joints_cost_fn(robot), params.center_joints_weight});
    }
    if (params.avoid_joint_limits_weight > 0) {
        goals.push_back(pick_ik::Goal{pick_ik::make_avoid_joint_limits_cost_fn(robot),
                                      params.center_joints_weight});
    }
    if (params.minimal_displacement_weight > 0) {
        goals.push_back(
            pick_ik::Goal{pick_ik::make_minimal_displacement_cost_fn(robot, initial_guess),
                          params.center_joints_weight});
    }

    // Make pose cost function
    auto const pose_cost_functions = pick_ik::make_pose_cost_functions({goal_frame},
                                                                       params.position_scale,
                                                                       params.rotation_scale);
    CHECK(pose_cost_functions.size() == 1);
    auto const cost_fn = pick_ik::make_cost_fn(pose_cost_functions, goals, fk_fn);

    // Make solution function
    auto const test_position = (params.position_scale > 0);
    std::optional<double> position_threshold = std::nullopt;
    if (test_position) {
        position_threshold = params.position_threshold;
    }
    auto const test_rotation = (params.rotation_scale > 0.0);
    std::optional<double> orientation_threshold = std::nullopt;
    if (test_rotation) {
        orientation_threshold = params.orientation_threshold;
    }
    auto const frame_tests =
        pick_ik::make_frame_tests({goal_frame}, position_threshold, orientation_threshold);
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk_fn);

    // Solve memetic IK
    return pick_ik::ik_memetic(initial_guess,
                               robot,
                               cost_fn,
                               solution_fn,
                               params.memetic_params,
                               params.approximate_solution,
                               params.print_debug);
}

TEST_CASE("Panda model Memetic IK") {
    using moveit::core::loadTestingRobotModel;
    auto const robot_model = loadTestingRobotModel("panda");

    auto const jmg = robot_model->getJointModelGroup("panda_arm");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"panda_hand"}).value();
    std::mutex mx;
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, mx, tip_link_indices);

    std::vector<double> const home_joint_angles =
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};

    SECTION("Panda model IK at home positions.") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto initial_guess = home_joint_angles;
        MemeticIkTestParams params;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.position_threshold));
    }

    SECTION("Panda model IK near home positions.") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto initial_guess = home_joint_angles;
        std::vector<double> const initial_guess_offsets = {0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1};
        for (size_t i = 0; i < initial_guess.size(); ++i) {
            initial_guess[i] += initial_guess_offsets[i];
        }
        MemeticIkTestParams params;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.position_threshold));
    }

    SECTION("Panda model IK at zero positions -- single threaded") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto const initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        MemeticIkTestParams params;
        params.print_debug = false;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.position_threshold));
    }

    SECTION("Panda model IK at zero positions -- multithreaded") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto const initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        MemeticIkTestParams params;
        params.memetic_params.num_threads = 4;
        params.print_debug = true;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.position_threshold));
    }

    SECTION("Panda model IK, with joint centering and limits avoiding.") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto const initial_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        MemeticIkTestParams params;
        params.center_joints_weight = 0.01;
        params.avoid_joint_limits_weight = 0.01;
        params.cost_threshold = 0.01;  // Need to raise this for joint centering
        params.position_threshold = 0.01;
        params.print_debug = false;

        auto const maybe_solution = solve_memetic_ik_test(robot_model,
                                                          "panda_arm",
                                                          "panda_hand",
                                                          goal_frame,
                                                          initial_guess,
                                                          params);

        REQUIRE(maybe_solution.has_value());
        auto const final_frame = fk_fn(maybe_solution.value())[0];
        CHECK(goal_frame.isApprox(final_frame, params.position_threshold));
    }
}
