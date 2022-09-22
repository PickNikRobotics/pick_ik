#include <pick_ik/math.hpp>

#include <catch2/catch_all.hpp>

TEST_CASE("pick_ik::clamp2") {
    SECTION("Does not clamp in range") {
        CHECK(pick_ik::clamp2(1.7, 0.0, 2.0) == Catch::Approx(1.7));
        CHECK(pick_ik::clamp2(0.1, 0.0, 2.0) == Catch::Approx(0.1));
        CHECK(pick_ik::clamp2(1.99, 0.0, 2.0) == Catch::Approx(1.99));
        CHECK(pick_ik::clamp2(1.1, 0.0, 2.0) == Catch::Approx(1.1));
    }

    SECTION("Clamps higher than high") {
        CHECK(pick_ik::clamp2(2.1, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(pick_ik::clamp2(10000.0, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(pick_ik::clamp2(1e45, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(pick_ik::clamp2(3.3, 0.0, 2.0) == Catch::Approx(2.0));
    }

    SECTION("Clamps lower than low") {
        CHECK(pick_ik::clamp2(-0.1, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(pick_ik::clamp2(-10000.0, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(pick_ik::clamp2(-1e300, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(pick_ik::clamp2(-3.3, 0.0, 2.0) == Catch::Approx(0.0));
    }
}
