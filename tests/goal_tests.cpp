#include <pick_ik/goal.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <Eigen/Geometry>
#include <cmath>

TEST_CASE("pick_ik::make_frame_tests") {
    auto const position_epsilon = 0.00001;
    auto const orientation_epsilon = 0.001;
    auto const zero_frame = Eigen::Isometry3d::Identity();

    SECTION("One goal, one test") {
        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.size() == 1);
    }

    SECTION("Zero threshold") {
        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, 0.0, 0.0);
        CHECK(test_fns.at(0)({zero_frame}) == true);
    }

    SECTION("Goal is almost frame, but not quite") {
        Eigen::Isometry3d const frame =
            Eigen::Translation3d(position_epsilon, position_epsilon, position_epsilon) *
            Eigen::Quaterniond(1 - orientation_epsilon, 0.0, 0.0, orientation_epsilon);

        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.at(0)({frame}) == false);
    }

    SECTION("Goal is almost frame, within position but not orientation threshold") {
        Eigen::Isometry3d const frame =
            Eigen::Translation3d(0.0, 0.000009, 0.0) * Eigen::Quaterniond(0.707, 0.0, 0.707, 0.0);

        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.at(0)({frame}) == false);
    }

    SECTION("Goal is almost frame, within threshold") {
        Eigen::Isometry3d const frame = Eigen::Translation3d(0.0, 0.000009, 0.0) *
                                        Eigen::Quaterniond(0.99999, 0.0, 0.0, 0.00001);

        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.at(0)({frame}) == true);
    }

    SECTION("Goal is frame") {
        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.at(0)({zero_frame}) == true);
    }

    SECTION("Goal is frame, but orientation is different") {
        auto const zero_frame_rotated = Eigen::Translation3d(0.0, 0.0, 0.0) *
                                        Eigen::AngleAxisd(M_PI_4, Eigen::Vector3d::UnitZ());

        // With rotation in frame test
        auto const test_fns =
            pick_ik::make_frame_tests({zero_frame}, position_epsilon, orientation_epsilon);
        CHECK(test_fns.at(0)({zero_frame_rotated}) == false);

        // Without rotation in frame test
        auto const test_fns_pos_only = pick_ik::make_frame_tests({zero_frame}, position_epsilon);
        CHECK(test_fns_pos_only.at(0)({zero_frame_rotated}) == true);
    }
}

TEST_CASE("pick_ik::make_pose_cost_fn") {
    auto const zero_frame = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d const translate_y2_frame =
        Eigen::Translation3d(0.0, 2.0, 0.0) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    Eigen::Isometry3d const translate_xy1_frame =
        Eigen::Translation3d(1.0, 1.0, 0.0) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    Eigen::Isometry3d const translate_xyz1_frame =
        Eigen::Translation3d(1.0, 1.0, 1.0) * Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    Eigen::Isometry3d const rotate_x1_frame =
        Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX());
    Eigen::Isometry3d const rotate_y2_frame =
        Eigen::Translation3d(0.0, 0.0, 0.0) * Eigen::AngleAxisd(2.0, Eigen::Vector3d::UnitY());

    SECTION("Goal is frame") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.0);
        CHECK(cost_fn({zero_frame}) == Catch::Approx(0.0));
    }

    SECTION("Goal is frame, with rotation scale") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(cost_fn({zero_frame}) == Catch::Approx(0.0));
    }

    SECTION("Goal is second index") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(translate_y2_frame, 1, 0.0);
        CHECK(cost_fn({zero_frame, translate_y2_frame}) == Catch::Approx(0.0));
    }

    SECTION("Translation along one axis, square of distance") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(cost_fn({translate_y2_frame}) == Catch::Approx(std::pow(2.0, 2)));
    }

    SECTION("Translation in two axes, square of distance") {
        // We know that the distance between (0,0,0) and (1,1,0) is sqrt(2),
        // so the squared distance in the cost function should be 2.
        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(const_fn({translate_xy1_frame}) == Catch::Approx(2.0));
    }

    SECTION("Translation in three axes, square of distance") {
        // We know that the distance between (0,0,0) and (1,1,1) is sqrt(3),
        // so the squared distance in the cost function should be 3.
        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(const_fn({translate_xyz1_frame}) == Catch::Approx(3.0));
    }

    SECTION("Zero rotation scale with rotation") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.0);
        CHECK(cost_fn({rotate_x1_frame}) == Catch::Approx(0.0));
    }

    SECTION("Negative rotation scale same as zero rotation scale") {
        auto const cost_fn_zero = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.0);
        auto const cost_fn_neg = pick_ik::make_pose_cost_fn(zero_frame, 0, -0.5);
        CHECK(cost_fn_zero({rotate_x1_frame}) == cost_fn_neg({rotate_x1_frame}));
    }

    SECTION("Rotation in one axis") {
        // Since we specified an angle of 2 radians about the Y axis, the
        // squared angle should be 4.0.
        auto const rotational_distance = 2.0;
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 1.0);
        CHECK(cost_fn({rotate_y2_frame}) == Catch::Approx(std::pow(rotational_distance, 2)));
    }

    SECTION("Rotation in one axis, scaled 0.5") {
        // Since we specified an angle of 2 radians about the Y axis, the
        // squared angle should be 4.0.
        auto const rotational_distance = 2.0;
        auto const rotation_scale = 0.5;

        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, rotation_scale);

        // The rotation scale is squared in addition to the square of the distance
        CHECK(cost_fn({rotate_y2_frame}) ==
              Catch::Approx(std::pow(rotational_distance, 2) * std::pow(rotation_scale, 2)));
    }

    SECTION("Test 0 - sample data taken from instrumenting bio_ik") {
        auto const q_goal = Eigen::Quaterniond(3.2004117980888137e-12,
                                               0.9239557003781338,
                                               -0.38249949508300274,
                                               1.324932598914536e-12);
        Eigen::Isometry3d const goal =
            Eigen::Translation3d(0.3548182547092438, -0.04776066541671753, 0.5902695655822754) *
            q_goal;

        auto const q_frame = Eigen::Quaterniond(-0.0033032628064278945,
                                                0.9163043570028795,
                                                -0.40044067474764505,
                                                -0.004762331364117075);
        Eigen::Isometry3d const frame =
            Eigen::Translation3d(0.3363926217416014, -0.043807946580255344, 0.5864240526436293) *
            q_frame;

        auto const rotation_scale = 0.5;
        auto const expected_cost =
            (goal.translation() - frame.translation()).squaredNorm() +
            std::pow(2.0 * std::acos(q_goal.dot(q_frame)) * rotation_scale, 2);

        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost = cost_fn({frame});
        CHECK(cost == Catch::Approx(expected_cost));
    }

    SECTION("Test 2 - sample data taken from instrumenting bio_ik") {
        auto const q_goal = Eigen::Quaterniond(3.2004117980888137e-12,
                                               0.9239557003781338,
                                               -0.38249949508300274,
                                               1.324932598914536e-12);
        Eigen::Isometry3d const goal =
            Eigen::Translation3d(0.3327501714229584, -0.025710120797157288, 0.5902695655822754) *
            q_goal;

        auto const q_frame = Eigen::Quaterniond(2.1223489422435532e-07,
                                                0.9239554647443051,
                                                -0.38250006378889556,
                                                1.925047999919496e-05);
        Eigen::Isometry3d const frame =
            Eigen::Translation3d(0.3327318727877646, -0.02570328270961634, 0.5900141633600922) *
            q_frame;

        auto const rotation_scale = 0.5;
        auto const expected_cost =
            (goal.translation() - frame.translation()).squaredNorm() +
            std::pow(2.0 * std::acos(q_goal.dot(q_frame)) * rotation_scale, 2);

        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost = cost_fn({frame});
        CHECK(cost == Catch::Approx(expected_cost));
    }
}

TEST_CASE("pick_ik::make_pose_cost_functions") {
    Eigen::Isometry3d const goal =
        Eigen::Translation3d(0.3327501714229584, -0.025710120797157288, 0.5902695655822754) *
        Eigen::Quaterniond(3.2004117980888137e-12,
                           0.9239557003781338,
                           -0.38249949508300274,
                           1.324932598914536e-12);
    Eigen::Isometry3d const frame =
        Eigen::Translation3d(0.3327318727877646, -0.02570328270961634, 0.5900141633600922) *
        Eigen::Quaterniond(2.1223489422435532e-07,
                           0.9239554647443051,
                           -0.38250006378889556,
                           1.925047999919496e-05);
    auto const rotation_scale = 0.5;

    SECTION("Function is same as pick_ik::make_pose_cost_fn") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal}, rotation_scale);

        CHECK(cost_fn({frame}) == cost_fns.at(0)({frame}));
    }

    SECTION("One goal, one function") {
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal}, rotation_scale);
        CHECK(cost_fns.size() == 1);
    }

    SECTION("Two goals, two functions") {
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal, frame}, rotation_scale);
        CHECK(cost_fns.size() == 2);
    }

    SECTION("First goal, tests first frame") {
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal, frame}, rotation_scale);
        CHECK(cost_fns.at(0)({goal, frame}) == Catch::Approx(0.0).margin(1e-15));
    }

    SECTION("Second goal, tests second frame") {
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal, frame}, rotation_scale);
        CHECK(cost_fns.at(1)({goal, frame}) == Catch::Approx(0.0).margin(1e-15));
    }
}
