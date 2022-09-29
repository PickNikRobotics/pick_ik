#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <iostream>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/utils/robot_model_test_utils.h>

auto make_rr_model_for_ik() {
    /**
     * We aim to make an RR robot like this:
     *
     *  revolute_2 -->  (b) ==== |E  <-- ee
     *                 //
     *                //
     *               (a)  <-- revolute_1
     *             origin
     **/
    auto builder = moveit::core::RobotModelBuilder("rr", "base");

    // Define transforms and joint axes
    geometry_msgs::msg::Pose origin;
    origin.orientation.w = 1.0;

    geometry_msgs::msg::Pose tform_x1;
    tform_x1.position.x = 1.0;  // This is the length of each link.
    tform_x1.orientation.w = 1.0;

    auto const z_axis = urdf::Vector3(0, 0, 1);

    // Build the actual robot chain.
    builder.addChain("base->a", "revolute", {origin}, z_axis);
    builder.addChain("a->b", "revolute", {tform_x1}, z_axis);
    builder.addChain("b->ee", "fixed", {tform_x1});
    builder.addGroupChain("base", "ee", "group");
    CHECK(builder.isValid());
    return builder.build();
}

TEST_CASE("RR model FK") {
    auto const robot_model = make_rr_model_for_ik();

    auto const jmg = robot_model->getJointModelGroup("group");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"ee"}).value();

    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, tip_link_indices);

    SECTION("Zero joint position") {
        std::vector<double> const joint_vals = {0.0, 0.0};
        auto const result = fk_fn(joint_vals);
        CHECK(result[0].translation().x() == Catch::Approx(2.0));
        CHECK(result[0].translation().y() == Catch::Approx(0.0));
    }

    SECTION("Non-zero joint position") {
        auto constexpr PI_4 = 0.78539816339;
        std::vector<double> const joint_vals = {PI_4, -PI_4};
        auto const result = fk_fn(joint_vals);
        CHECK(result[0].translation().x() == Catch::Approx(1.707).margin(0.001));
        CHECK(result[0].translation().y() == Catch::Approx(0.707).margin(0.001));
    }
}

TEST_CASE("RR model IK") {
    auto const robot_model = make_rr_model_for_ik();
    auto const jmg = robot_model->getJointModelGroup("group");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"ee"}).value();
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, tip_link_indices);

    SECTION("Pose goal only") {
        // Define goal frame and expected joint values.
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(2.0, 0.0, 0.0) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
        const std::vector<double> expected_joint_angles = {0.0, 0.0};

        // Make solution function
        auto const twist_threshold = 0.001;
        auto const frame_tests = pick_ik::make_frame_tests({goal_frame}, twist_threshold);
        CHECK(frame_tests.size() == 1);

        auto const cost_function = kinematics::KinematicsBase::IKCostFn();  // TODO

        geometry_msgs::msg::Pose ik_pose;
        ik_pose.position.x = 2.0;
        ik_pose.orientation.w = 1.0;

        auto const ik_seed_state = {0.1, -0.1};  // Initial guess
        auto const cost_threshold = 0.001;
        std::vector<pick_ik::Goal> goals = {};

        auto const solution_fn =
            pick_ik::make_is_solution_test_fn(frame_tests, goals, cost_threshold, fk_fn);

        CHECK(solution_fn({0.0, 0.0}) == true);  // Exact match (within floating point tolerance)
        CHECK(solution_fn({0.0001, -0.0001}) == true);  // Match within threshold
        CHECK(solution_fn({0.01, -0.01}) == false);     // Match outside threshold
        CHECK(solution_fn({1.5707, 0.0}) == false);     // Not a match

        // Make pose cost function
        auto const rotation_scale = 0.5;
        auto const pose_cost_functions =
            pick_ik::make_pose_cost_functions({goal_frame}, rotation_scale);
        CHECK(pose_cost_functions.size() == 1);

        // Solve IK
        auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);
        auto const cost_fn = pick_ik::make_cost_fn(pose_cost_functions, goals, fk_fn);

        auto const timeout = 10.0;
        auto const return_approx_solution = false;
        auto const maybe_solution = pick_ik::ik_search(ik_seed_state,
                                                       robot,
                                                       cost_fn,
                                                       solution_fn,
                                                       timeout,
                                                       return_approx_solution);
        CHECK(maybe_solution);
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }
}
