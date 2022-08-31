#include <gd_ik/algorithm.hpp>

#include <catch2/catch_all.hpp>

TEST_CASE("gd_ik::select_indexes") {
    auto const input = std::vector<int>{0, 1, 2};
    SECTION("Select single index results in sub-sampled vector") {
        auto res = gd_ik::select_indexes(input, std::vector<size_t>{1});
        CHECK(res.size() == 1);
        CHECK(res.at(0) == 1);
    }

    SECTION("Select multiple results in sub-sampled vector") {
        auto res = gd_ik::select_indexes(input, std::vector<size_t>{0, 2});
        CHECK(res.size() == 2);
        CHECK(res.at(0) == 0);
        CHECK(res.at(1) == 2);
    }

    SECTION("Select empty results in empty vector") {
        auto res = gd_ik::select_indexes(input, std::vector<size_t>{});
        CHECK(res.size() == 0);
    }

    SECTION("Select invalid indexes throws") {
        CHECK_THROWS(gd_ik::select_indexes(input, std::vector<size_t>{4}));
    }
}

TEST_CASE("gd_ik::set_indexes") {
    auto const input = std::vector<int>{0, 1, 2};
    SECTION("Not setting any values returns original input") {
        auto res = gd_ik::set_indexes(input, std::vector<int>{}, std::vector<size_t>{});
        CHECK(res == input);
    }

    SECTION("Setting one value sets only that value") {
        auto res = gd_ik::set_indexes(input, std::vector<int>{-1}, std::vector<size_t>{0});
        CHECK(res.size() == input.size());
        CHECK(res.at(0) == -1);
        CHECK(res.at(1) == 1);
        CHECK(res.at(2) == 2);
    }

    SECTION("Setting last two values set only them and leave first alone") {
        auto res = gd_ik::set_indexes(input, std::vector<int>{-1, -2}, std::vector<size_t>{1, 2});
        CHECK(res.size() == input.size());
        CHECK(res.at(0) == 0);
        CHECK(res.at(1) == -1);
        CHECK(res.at(2) == -2);
    }

    SECTION("Setting bad index value throws") {
        CHECK_THROWS(gd_ik::set_indexes(input, std::vector<int>{-1}, std::vector<size_t>{4}));
    }

    SECTION("Not enough indexes throws") {
        CHECK_THROWS(gd_ik::set_indexes(input, std::vector<int>{1}, std::vector<size_t>{}));
    }
}
