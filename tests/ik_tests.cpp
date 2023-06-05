#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Geometry>
#include <cmath>
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
    tform_x1.position.x = 1.0;
    tform_x1.orientation.w = 1.0;

    geometry_msgs::msg::Pose tform_x2;
    tform_x2.position.x = 2.0;
    tform_x2.orientation.w = 1.0;

    auto const z_axis = urdf::Vector3(0, 0, 1);

    // Build the actual robot chain.
    builder.addChain("base->a", "revolute", {origin}, z_axis);
    builder.addChain("a->b", "revolute", {tform_x2}, z_axis);
    builder.addChain("b->ee", "fixed", {tform_x1});
    builder.addGroupChain("base", "ee", "group");
    CHECK(builder.isValid());
    return builder.build();
}

TEST_CASE("RR model FK") {
    auto const robot_model = make_rr_model_for_ik();

    auto const jmg = robot_model->getJointModelGroup("group");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"ee"}).value();

    std::mutex mx;
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, mx, tip_link_indices);

    SECTION("Zero joint position") {
        std::vector<double> const joint_vals = {0.0, 0.0};
        auto const result = fk_fn(joint_vals);
        CHECK(result[0].translation().x() == Catch::Approx(3.0));
        CHECK(result[0].translation().y() == Catch::Approx(0.0));
    }

    SECTION("Non-zero joint position") {
        std::vector<double> const joint_vals = {M_PI_4, -M_PI_4};
        auto const expected_x = 2.0 * std::cos(M_PI_4) + 1.0;
        auto const expected_y = 2.0 * std::sin(M_PI_4);

        auto const result = fk_fn(joint_vals);
        CHECK(result[0].translation().x() == Catch::Approx(expected_x).margin(0.001));
        CHECK(result[0].translation().y() == Catch::Approx(expected_y).margin(0.001));
    }
}

// Helper param struct and function to test IK solution.
struct IkTestParams {
    double position_threshold = 0.0001;
    double orientation_threshold = 0.001;
    double cost_threshold = 0.0001;
    double position_scale = 1.0;
    double rotation_scale = 1.0;
    bool return_approximate_solution = false;
    pick_ik::GradientIkParams gd_params;
};

auto solve_ik_test(moveit::core::RobotModelPtr robot_model,
                   std::string const group_name,
                   std::string const goal_frame_name,
                   Eigen::Isometry3d const& goal_frame,
                   std::vector<double> const& initial_guess,
                   IkTestParams const& params = IkTestParams())
    -> std::optional<std::vector<double>> {
    // Make forward kinematics function
    auto const jmg = robot_model->getJointModelGroup(group_name);
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {goal_frame_name}).value();
    std::mutex mx;
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, mx, tip_link_indices);

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
    auto const cost_function =
        kinematics::KinematicsBase::IKCostFn();  // What should be instantiated here?
    std::vector<pick_ik::Goal> goals = {};       // TODO: Only works if empty.
    auto const solution_fn =
        pick_ik::make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk_fn);

    // Make pose cost function
    auto const pose_cost_functions = pick_ik::make_pose_cost_functions({goal_frame},
                                                                       params.position_scale,
                                                                       params.rotation_scale);
    CHECK(pose_cost_functions.size() == 1);

    // Solve IK
    auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);
    auto const cost_fn = pick_ik::make_cost_fn(pose_cost_functions, goals, fk_fn);
    return pick_ik::ik_gradient(initial_guess,
                                robot,
                                cost_fn,
                                solution_fn,
                                params.gd_params,
                                params.return_approximate_solution);
}

TEST_CASE("RR model IK") {
    auto const robot_model = make_rr_model_for_ik();

    SECTION("Zero joint angles with close initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};
        std::vector<double> const initial_guess = {0.1, -0.1};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Zero joint angles with far initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(3.0, 0.0, 0.0) * Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};
        std::vector<double> const initial_guess = {M_PI_2, -M_PI_2};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Nonzero joint angles with near initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitZ());
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};
        std::vector<double> const initial_guess = {M_PI_4 + 0.1, M_PI_2 - 0.1};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Nonzero joint angles with far initial guess") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::AngleAxisd(0.75 * M_PI, Eigen::Vector3d::UnitZ());
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        REQUIRE(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }

    SECTION("Unreachable position") {
        auto const goal_frame = Eigen::Isometry3d::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};  // Doesn't matter
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        CHECK(!maybe_solution.has_value());
    }

    SECTION("Reachable position, but not orientation") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {0.0, 0.0};  // Doesn't matter
        std::vector<double> const initial_guess = {0.0, 0.0};

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess);

        CHECK(!maybe_solution.has_value());
    }

    SECTION("Reachable position, but not orientation -- zero rotation scale") {
        Eigen::Isometry3d const goal_frame =
            Eigen::Translation3d(std::sin(M_PI_4), 3.0 * std::sin(M_PI_4), 0.0) *
            Eigen::Quaterniond::Identity();
        std::vector<double> const expected_joint_angles = {M_PI_4, M_PI_2};  // Doesn't matter
        std::vector<double> const initial_guess = {M_PI_4 + 0.1, M_PI_2 - 0.1};
        auto params = IkTestParams();
        params.rotation_scale = 0.0;

        auto const maybe_solution =
            solve_ik_test(robot_model, "group", "ee", goal_frame, initial_guess, params);

        CHECK(maybe_solution.has_value());
        CHECK(maybe_solution.value()[0] == Catch::Approx(expected_joint_angles[0]).margin(0.01));
        CHECK(maybe_solution.value()[1] == Catch::Approx(expected_joint_angles[1]).margin(0.01));
    }
}

TEST_CASE("Panda model IK") {
    using moveit::core::loadTestingRobotModel;
    auto const robot_model = loadTestingRobotModel("panda");

    auto const jmg = robot_model->getJointModelGroup("panda_arm");
    auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"panda_hand"}).value();
    std::mutex mx;
    auto const fk_fn = pick_ik::make_fk_fn(robot_model, jmg, mx, tip_link_indices);

    std::vector<double> const home_joint_angles =
        {0.0, -M_PI_4, 0.0, -3.0 * M_PI_4, 0.0, M_PI_2, M_PI_4};

    SECTION("Panda model IK at exact home values") {
        auto const goal_frame = fk_fn(home_joint_angles)[0];
        auto const initial_guess = home_joint_angles;
        auto params = IkTestParams();
        params.rotation_scale = 0.5;

        auto const maybe_solution = solve_ik_test(robot_model,
                                                  "panda_arm",
                                                  "panda_hand",
                                                  goal_frame,
                                                  initial_guess,
                                                  params);

        REQUIRE(maybe_solution.has_value());
        for (size_t i = 0; i < initial_guess.size(); ++i) {
            CHECK(maybe_solution.value()[i] == Catch::Approx(home_joint_angles[i]).margin(0.01));
        }
    }

    SECTION("Panda model IK at perturbed home values") {
        std::vector<double> const actual_joint_angles =
            {0.1, -M_PI_4 - 0.1, 0.1, -3.0 * M_PI_4 - 0.1, 0.1, M_PI_2 - 0.1, M_PI_4 + 0.1};
        auto const goal_frame = fk_fn(actual_joint_angles)[0];

        auto const initial_guess = home_joint_angles;
        auto params = IkTestParams();
        params.rotation_scale = 0.5;

        auto const maybe_solution = solve_ik_test(robot_model,
                                                  "panda_arm",
                                                  "panda_hand",
                                                  goal_frame,
                                                  initial_guess,
                                                  params);

        REQUIRE(maybe_solution.has_value());
        for (size_t i = 0; i < initial_guess.size(); ++i) {
            // Note the extra tolerance...
            CHECK(maybe_solution.value()[i] == Catch::Approx(actual_joint_angles[i]).margin(0.025));
        }
    }
}
