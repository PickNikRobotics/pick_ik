#include <pick_ik/frame.hpp>
#include <pick_ik/goal.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

TEST_CASE("pick_ik::make_frame_tests") {
    auto const epsilon = 0.00001;
    auto const zero_frame = pick_ik::Frame{
        tf2::Vector3(0, 0, 0),
        tf2::Quaternion(0, 0, 0, 1),
    };

    SECTION("One goal, one test") {
        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, epsilon);
        CHECK(test_fns.size() == 1);
    }

    SECTION("Zero threshold") {
        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, 0.);
        CHECK(test_fns.at(0)({zero_frame}) == false);
    }

    SECTION("Goal is almost frame, but not quite") {
        auto const frame = pick_ik::Frame{
            tf2::Vector3(epsilon, epsilon, epsilon),
            tf2::Quaternion(0, 0, epsilon, 1 - epsilon),
        };

        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, epsilon);
        CHECK(test_fns.at(0)({frame}) == false);
    }

    SECTION("Goal is almost frame, within threshold") {
        auto const frame = pick_ik::Frame{
            tf2::Vector3(0, 0.000009, 0),
            tf2::Quaternion(0, 0, 0.000001, 0.99999),
        };

        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, epsilon);
        CHECK(test_fns.at(0)({frame}) == true);
    }

    SECTION("Goal is frame") {
        auto const test_fns = pick_ik::make_frame_tests({zero_frame}, epsilon);
        CHECK(test_fns.at(0)({zero_frame}) == true);
    }
}

TEST_CASE("pick_ik::make_pose_cost_fn") {
    auto const zero_frame = pick_ik::Frame{
        tf2::Vector3(0, 0, 0),
        tf2::Quaternion(0, 0, 0, 1),
    };
    auto const translate_y2_frame = pick_ik::Frame{
        tf2::Vector3(0, 2, 0),
        tf2::Quaternion(0, 0, 0, 1),
    };
    auto const translate_xy1_frame = pick_ik::Frame{
        tf2::Vector3(1, 1, 0),
        tf2::Quaternion(0, 0, 0, 1),
    };
    auto const translate_xyz1_frame = pick_ik::Frame{
        tf2::Vector3(1, 1, 1),
        tf2::Quaternion(0, 0, 0, 1),
    };
    auto const rotate_x1_frame = pick_ik::Frame{
        tf2::Vector3(0, 0, 0),
        tf2::Quaternion(1, 0, 0, 1),
    };
    auto const rotate_y2_frame = pick_ik::Frame{
        tf2::Vector3(0, 0, 0),
        tf2::Quaternion(0, 2, 0, 1),
    };

    SECTION("Goal is frame") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0);
        CHECK(cost_fn({zero_frame}) == Catch::Approx(0.));
    }

    SECTION("Goal is frame, with rotation scale") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(cost_fn({zero_frame}) == Catch::Approx(0.));
    }

    SECTION("Goal is second index") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(translate_y2_frame, 1, 0);
        CHECK(cost_fn({zero_frame, translate_y2_frame}) == Catch::Approx(0.));
    }

    SECTION("Translation along one axis square of distance") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(cost_fn({translate_y2_frame}) == Catch::Approx(std::pow(2., 2)));
    }

    SECTION("Zero rotation scale with rotation") {
        auto const cost_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0);
        CHECK(cost_fn({rotate_x1_frame}) == Catch::Approx(0.));
    }

    SECTION("Negative rotation same as zero rotation scale") {
        auto const cost_fn_zero = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.);
        auto const cost_fn_neg = pick_ik::make_pose_cost_fn(zero_frame, 0, -0.5);
        CHECK(cost_fn_zero({rotate_x1_frame}) == cost_fn_neg({rotate_x1_frame}));
    }

    SECTION("Translation in two axis, square of distance") {
        // We know that the distance between (0,0,0) and (1,1,0) is sqrt(2)
        auto const translation_distance = std::sqrt(2.);

        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(const_fn({translate_xy1_frame}) == Catch::Approx(std::pow(translation_distance, 2)));
    }

    SECTION("Translation in three axis, square of distance") {
        // We know that the distance between (0,0,0) and (1,1,1) is sqrt(3)
        auto const translation_distance = std::sqrt(3.);

        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 0.5);
        CHECK(const_fn({translate_xyz1_frame}) == Catch::Approx(std::pow(translation_distance, 2)));
    }

    SECTION("Rotation in one axis") {
        // We know that the distance between (0, 0, 0, 1) and (2, 0, 0, 1) is 2
        auto const rotational_distance = 2.;

        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, 1.0);
        CHECK(const_fn({rotate_y2_frame}) == Catch::Approx(std::pow(rotational_distance, 2)));
    }

    SECTION("Rotation in one axis, scaled 0.5") {
        // We know that the distance between (0, 0, 0, 1) and (2, 0, 0, 1) is 2
        auto const rotational_distance = 2.;
        auto const rotation_scale = 0.5;

        auto const const_fn = pick_ik::make_pose_cost_fn(zero_frame, 0, rotation_scale);

        // The rotation scale is squared before being multiplied by the square of the distance
        CHECK(const_fn({rotate_y2_frame}) ==
              Catch::Approx(std::pow(rotational_distance, 2) * std::pow(rotation_scale, 2)));
    }

    SECTION("Test 0 - sample data taken from instrumenting bio_ik") {
        auto const goal = pick_ik::Frame{
            tf2::Vector3(0.3548182547092438, -0.04776066541671753, 0.5902695655822754),
            tf2::Quaternion(0.9239557003781338,
                            -0.38249949508300274,
                            1.324932598914536e-12,
                            3.2004117980888137e-12)};
        auto const frame = pick_ik::Frame{
            tf2::Vector3(0.3363926217416014, -0.043807946580255344, 0.5864240526436293),
            tf2::Quaternion(0.9163043570028795,
                            -0.40044067474764505,
                            -0.004762331364117075,
                            -0.0033032628064278945)};
        auto const rotation_scale = 0.5;
        auto const error = 0.00047342098832687954;

        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost = cost_fn(std::vector<pick_ik::Frame>{frame});
        CHECK(cost == Catch::Approx(error));
    }
    SECTION("Test 1 - sample data taken from instrumenting bio_ik") {
        auto const goal = pick_ik::Frame{
            tf2::Vector3(0.3548182547092438, -0.04776066541671753, 0.5902695655822754),
            tf2::Quaternion(0.9239557003781338,
                            -0.38249949508300274,
                            1.324932598914536e-12,
                            3.2004117980888137e-12)};
        auto const frame = pick_ik::Frame{
            tf2::Vector3(0.3548171636285567, -0.04776037000115037, 0.590266184369505),
            tf2::Quaternion(0.9239556600447023,
                            -0.3824995924622168,
                            5.939810921340453e-06,
                            -1.521495969650019e-06)};
        auto const rotation_scale = 0.5;
        auto const error = 2.2112179037775748e-11;

        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost = cost_fn(std::vector<pick_ik::Frame>{frame});
        CHECK(cost == Catch::Approx(error));
    }
    SECTION("Test 2 - sample data taken from instrumenting bio_ik") {
        auto const goal = pick_ik::Frame{
            tf2::Vector3(0.3327501714229584, -0.025710120797157288, 0.5902695655822754),
            tf2::Quaternion(0.9239557003781338,
                            -0.38249949508300274,
                            1.324932598914536e-12,
                            3.2004117980888137e-12)};
        auto const frame = pick_ik::Frame{
            tf2::Vector3(0.3327318727877646, -0.02570328270961634, 0.5900141633600922),
            tf2::Quaternion(0.9239554647443051,
                            -0.38250006378889556,
                            1.925047999919496e-05,
                            2.1223489422435532e-07)};
        auto const rotation_scale = 0.5;
        auto const error = 6.570464581759838e-08;

        auto const cost_fn = pick_ik::make_pose_cost_fn(goal, 0, rotation_scale);
        auto const cost = cost_fn(std::vector<pick_ik::Frame>{frame});
        CHECK(cost == Catch::Approx(error));
    }
}

TEST_CASE("pick_ik::make_pose_cost_functions") {
    auto const goal =
        pick_ik::Frame{tf2::Vector3(0.3327501714229584, -0.025710120797157288, 0.5902695655822754),
                       tf2::Quaternion(0.9239557003781338,
                                       -0.38249949508300274,
                                       1.324932598914536e-12,
                                       3.2004117980888137e-12)};
    auto const frame =
        pick_ik::Frame{tf2::Vector3(0.3327318727877646, -0.02570328270961634, 0.5900141633600922),
                       tf2::Quaternion(0.9239554647443051,
                                       -0.38250006378889556,
                                       1.925047999919496e-05,
                                       2.1223489422435532e-07)};
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
        CHECK(cost_fns.at(0)({goal, frame}) == Catch::Approx(0.));
    }

    SECTION("Second goal, tests second frame") {
        auto const cost_fns = pick_ik::make_pose_cost_functions({goal, frame}, rotation_scale);
        CHECK(cost_fns.at(1)({goal, frame}) == Catch::Approx(0.));
    }
}
