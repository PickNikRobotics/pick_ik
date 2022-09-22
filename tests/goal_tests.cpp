#include <pick_ik/frame.hpp>
#include <pick_ik/goal.hpp>

#include <catch2/catch_all.hpp>

TEST_CASE("pick_ik::make_pose_cost_fn") {
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
