#include <range/v3/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tl_expected/expected.hpp>

#include <cstdio>
#include <fmt/core.h>
#include <fmt/ranges.h>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>
#include <variant>
#include <vector>

using geometry_msgs::msg::Pose;
using moveit::planning_interface::MoveGroupInterface;

auto const kBioIkPlugin = "bio_ik/BioIKKinematicsPlugin";
auto const kGdIkPlugin = "gd_ik/GDIKPlugin";
auto const kGroupName = "panda_arm";
auto const kKdlPlugin = "kdl_kinematics_plugin/KDLKinematicsPlugin";
auto const kNodeName = "ik_jogging_test";
auto const kLogger = rclcpp::get_logger(kNodeName);
auto const kStartPose = [] {
    Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
}();
// found using to_joint_angles from kHomePose
auto const kReadyAngles = std::vector<double>{
    0,
    -0.785,
    0,
    2.356,
    0,
    1.571,
    0.785,
};

struct IKSolveFailed {};
using Error = std::variant<IKSolveFailed>;

#define TRY(m)                                                         \
    ({                                                                 \
        auto const& exp = (m);                                         \
        if (!exp.has_value()) return tl::make_unexpected(exp.error()); \
        exp.value();                                                   \
    })

std::string to_string(std::vector<std::string> const& joint_names,
                      std::vector<double> const& joint_angles);

Pose operator+(Pose const& lhs, Pose const& rhs) {
    Pose ret;
    ret.orientation.x = lhs.orientation.x + rhs.orientation.x;
    ret.orientation.y = lhs.orientation.y + rhs.orientation.y;
    ret.orientation.z = lhs.orientation.z + rhs.orientation.z;
    ret.orientation.w = lhs.orientation.w + rhs.orientation.w;
    ret.position.x = lhs.position.x + rhs.position.x;
    ret.position.y = lhs.position.y + rhs.position.y;
    ret.position.z = lhs.position.z + rhs.position.z;
    return ret;
}

tl::expected<std::vector<double>, Error> to_joint_angles(MoveGroupInterface& move_group_interface,
                                                         std::string const& group_name,
                                                         Pose const& pose) {
    auto const robot_model = move_group_interface.getRobotModel();
    auto const* jmg = robot_model->getJointModelGroup(group_name);
    std::vector<std::string> link_model_tips;
    jmg->getEndEffectorTips(link_model_tips);
    auto const link_model_tip = link_model_tips.at(0);

    auto robot_state = move_group_interface.getCurrentState();
    auto const success = robot_state->setFromIK(jmg, pose, link_model_tip, 0.1);
    if (!success) {
        return tl::make_unexpected(IKSolveFailed{});
    }
    robot_state->updateLinkTransforms();

    std::vector<double> joint_angles;
    robot_state->copyJointGroupPositions(jmg, joint_angles);
    return joint_angles;
}

void plan_and_move_to_position(MoveGroupInterface& move_group_interface,
                               std::vector<double> const& joint_angles) {
    move_group_interface.setJointValueTarget(joint_angles);

    // Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

    // Execute the plan
    if (success) {
        move_group_interface.execute(plan);
    } else {
        RCLCPP_ERROR(kLogger, "Planing failed!");
    }
}

void move_to_named_target(MoveGroupInterface& move_group_interface, std::string const& name) {
    move_group_interface.setNamedTarget(name);
    move_group_interface.move();
}

trajectory_msgs::msg::JointTrajectoryPoint to_point(
    std::vector<double> const& positions,
    rclcpp::Duration const& time_from_start = rclcpp::Duration::from_seconds(0.0)) {
    trajectory_msgs::msg::JointTrajectoryPoint msg;
    msg.positions = positions;
    msg.time_from_start = time_from_start;
    return msg;
}

tl::expected<moveit_msgs::msg::RobotTrajectory, Error> to_trajectory(
    MoveGroupInterface& move_group_interface,
    std::string const& group_name,
    std::vector<double> const& start_angles,
    Pose target_pose) {
    auto const robot_model = move_group_interface.getRobotModel();
    auto const* jmg = robot_model->getJointModelGroup(group_name);
    auto const target_angles = TRY(to_joint_angles(move_group_interface, group_name, target_pose));

    auto joint_names = jmg->getJointModelNames();

    fmt::print(stderr, "start: {}\n", to_string(joint_names, start_angles));
    fmt::print(stderr, "target: {}\n", to_string(joint_names, target_angles));

    moveit_msgs::msg::RobotTrajectory trajectory;
    trajectory.joint_trajectory.joint_names = jmg->getJointModelNames();
    trajectory.joint_trajectory.points = {
        to_point(start_angles),
        to_point(target_angles, rclcpp::Duration(0, 10'000'000)),
    };
    return trajectory;
}

std::vector<std::string> get_joint_names(MoveGroupInterface const& move_group_interface,
                                         std::string const& group_name) {
    auto const robot_model = move_group_interface.getRobotModel();
    auto const* jmg = robot_model->getJointModelGroup(group_name);
    return jmg->getJointModelNames();
}

std::string to_string(std::vector<std::string> const& joint_names,
                      std::vector<double> const& joint_angles) {
    return fmt::format(
        "{}",
        fmt::join(ranges::views::zip_with(
                      [](auto name, auto angle) { return fmt::format("[{}: {}]", name, angle); },
                      joint_names,
                      joint_angles),
                  ", "));
}

int main(int argc, char* argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        kNodeName,
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    node->declare_parameter("robot_description_kinematics.panda_arm.kinematics_solver",
                            kGdIkPlugin);
    node->set_parameters({
        rclcpp::Parameter("robot_description_kinematics.panda_arm.kinematics_solver", kGdIkPlugin),
    });

    auto move_group_interface = MoveGroupInterface(node, kGroupName);
    move_group_interface.startStateMonitor();

    // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    move_to_named_target(move_group_interface, "ready");

    auto start_angles = move_group_interface.getCurrentJointValues();
    auto start_pose = move_group_interface.getCurrentPose().pose;
    auto const joint_names = get_joint_names(move_group_interface, kGroupName);

    Pose delta_pose;
    delta_pose.position.x = 0.01;

    for (size_t i = 0; i < 10; ++i) {
        auto target_pose = start_pose + delta_pose;
        auto const trajectory =
            to_trajectory(move_group_interface, kGroupName, start_angles, target_pose);
        if (!trajectory) {
            break;
        }

        move_group_interface.setStartStateToCurrentState();
        auto const success = move_group_interface.execute(trajectory.value());

        if (!success) {
            break;
        }

        start_angles = trajectory.value().joint_trajectory.points.at(1).positions;
        start_pose = target_pose;
    }

    // Shutdown ROS
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
