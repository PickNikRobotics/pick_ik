#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/utils/robot_model_test_utils.h>

auto make_rr_model() {
    auto builder = moveit::core::RobotModelBuilder("rr", "base");
    geometry_msgs::msg::Pose origin;
    origin.position.x = 1;
    origin.orientation.w = 1;
    builder.addChain("base->a", "revolute", {origin}, urdf::Vector3(1, 0, 0));
    builder.addChain("a->b", "revolute", {origin}, urdf::Vector3(1, 0, 0));
    builder.addGroupChain("base", "b", "group");
    CHECK(builder.isValid());
    return builder.build();
}

TEST_CASE("pick_ik::get_link_indexes") {
    auto const robot_model = make_rr_model();

    SECTION("tip joint") {
        auto const tip_link_indexes = pick_ik::get_link_indexes(robot_model, {"b"});
        REQUIRE(tip_link_indexes.has_value());
        CHECK(tip_link_indexes->size() == 1);
        CHECK(tip_link_indexes->at(0) == 2);
    }

    SECTION("no joints") {
        auto const tip_link_indexes = pick_ik::get_link_indexes(robot_model, {});
        REQUIRE(tip_link_indexes.has_value());
        CHECK(tip_link_indexes->size() == 0);
    }

    SECTION("invalid joint") {
        auto const tip_link_indexes = pick_ik::get_link_indexes(robot_model, {"c"});
        REQUIRE(tip_link_indexes.has_value() == false);
    }

    SECTION("base joint") {
        auto const tip_link_indexes = pick_ik::get_link_indexes(robot_model, {"base"});
        REQUIRE(tip_link_indexes.has_value());
        CHECK(tip_link_indexes->size() == 1);
        CHECK(tip_link_indexes->at(0) == 0);
    }
}
