#include <pick_ik/robot.hpp>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <moveit/utils/robot_model_test_utils.h>

auto make_rr_model() {
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

TEST_CASE("pick_ik::get_link_indices") {
    auto const robot_model = make_rr_model();

    SECTION("second link") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"b"});
        REQUIRE(tip_link_indices.has_value());
        CHECK(tip_link_indices->size() == 1);
        CHECK(tip_link_indices->at(0) == 2);
    }

    SECTION("end effector link") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"ee"});
        REQUIRE(tip_link_indices.has_value());
        CHECK(tip_link_indices->size() == 1);
        CHECK(tip_link_indices->at(0) == 3);
    }

    SECTION("multiple links") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"a", "b", "ee"});
        REQUIRE(tip_link_indices.has_value());
        CHECK(tip_link_indices->size() == 3);
        CHECK(tip_link_indices->at(0) == 1);
        CHECK(tip_link_indices->at(1) == 2);
        CHECK(tip_link_indices->at(2) == 3);
    }

    SECTION("no joints") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {});
        REQUIRE(tip_link_indices.has_value());
        CHECK(tip_link_indices->size() == 0);
    }

    SECTION("invalid joint") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"c"});
        REQUIRE(tip_link_indices.has_value() == false);
    }

    SECTION("base joint") {
        auto const tip_link_indices = pick_ik::get_link_indices(robot_model, {"base"});
        REQUIRE(tip_link_indices.has_value());
        CHECK(tip_link_indices->size() == 1);
        CHECK(tip_link_indices->at(0) == 0);
    }
}

TEST_CASE("pick_ik::Robot::from -- Simple RR Model") {
    auto const robot_model = make_rr_model();
    auto* const jmg = robot_model->getJointModelGroup("group");
    auto const tip_link_indices =
        pick_ik::get_link_indices(robot_model, {"ee"})
            .or_else([](auto const& error) { throw std::invalid_argument(error); })
            .value();
    auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);

    SECTION("RR robot has two joints") { CHECK(robot.variables.size() == 2); }
}

TEST_CASE("pick_ik::Robot::from -- Panda Model") {
    using moveit::core::loadTestingRobotModel;
    auto const robot_model = loadTestingRobotModel("panda");
    auto* const jmg = robot_model->getJointModelGroup("panda_arm");
    auto const tip_link_indices =
        pick_ik::get_link_indices(robot_model, {"panda_hand"})
            .or_else([](auto const& error) { throw std::invalid_argument(error); })
            .value();
    auto const robot = pick_ik::Robot::from(robot_model, jmg, tip_link_indices);

    SECTION("Panda has seven joints") { CHECK(robot.variables.size() == 7); }
}
