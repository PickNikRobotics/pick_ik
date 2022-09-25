#include <pick_ik/math.hpp>

#include <catch2/catch_all.hpp>

namespace pick_ik {

TEST_CASE("clamp2") {
    SECTION("Does not clamp in range") {
        CHECK(clamp2(1.7, 0.0, 2.0) == Catch::Approx(1.7));
        CHECK(clamp2(0.1, 0.0, 2.0) == Catch::Approx(0.1));
        CHECK(clamp2(1.99, 0.0, 2.0) == Catch::Approx(1.99));
        CHECK(clamp2(1.1, 0.0, 2.0) == Catch::Approx(1.1));
    }

    SECTION("Clamps higher than high") {
        CHECK(clamp2(2.1, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(clamp2(10000.0, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(clamp2(1e45, 0.0, 2.0) == Catch::Approx(2.0));
        CHECK(clamp2(3.3, 0.0, 2.0) == Catch::Approx(2.0));
    }

    SECTION("Clamps lower than low") {
        CHECK(clamp2(-0.1, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(clamp2(-10000.0, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(clamp2(-1e300, 0.0, 2.0) == Catch::Approx(0.0));
        CHECK(clamp2(-3.3, 0.0, 2.0) == Catch::Approx(0.0));
    }
}

}  // namespace pick_ik
