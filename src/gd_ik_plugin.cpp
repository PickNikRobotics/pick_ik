#include <gd_ik/algorithm.hpp>
#include <gd_ik/fk_moveit.hpp>
#include <gd_ik/frame.hpp>
#include <gd_ik/goal.hpp>
#include <gd_ik/ik_gradient.hpp>
#include <gd_ik/robot.hpp>

#include <gd_ik_parameters.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <vector>

namespace gd_ik {
namespace {
auto const LOGGER = rclcpp::get_logger("gd_ik");
}

class GDIKPlugin : public kinematics::KinematicsBase {
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ParamListener> parameter_listener_;
    moveit::core::JointModelGroup const* jmg_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    Robot robot_;
    std::vector<size_t> tip_link_indexes_;
    std::vector<size_t> active_variable_indexes_;
    std::vector<double> minimal_displacement_factors_;

   public:
    virtual bool initialize(rclcpp::Node::SharedPtr const& node,
                            moveit::core::RobotModel const& robot_model,
                            std::string const& group_name, std::string const& base_frame,
                            std::vector<std::string> const& tip_frames,
                            double search_discretization) {
        node_ = node;
        parameter_listener_ = std::make_shared<ParamListener>(
            node, std::string("robot_description_kinematics.").append(group_name));

        // Initialize internal state of base class KinematicsBase
        // Creates these internal state variables:
        // robot_model_ <- shared_ptr to RobotModel
        // robot_description_ <- empty string
        // group_name_ <- group_name string
        // base_frame_ <- base_frame without leading /
        // tip_frames_ <- tip_frames without leading /
        // redundant_joint_discretization_ <- vector initialized with
        // search_discretization
        storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

        // Initialize internal state
        jmg_ = robot_model_->getJointModelGroup(group_name);
        if (!jmg_) {
            RCLCPP_ERROR(LOGGER, "failed to get joint model group %s", group_name.c_str());
            return false;
        }

        // Joint names come from jmg
        for (auto* joint_model : jmg_->getJointModels()) {
            if (joint_model->getName() != base_frame_ &&
                joint_model->getType() != moveit::core::JointModel::UNKNOWN &&
                joint_model->getType() != moveit::core::JointModel::FIXED) {
                joint_names_.push_back(joint_model->getName());
            }
        }

        // If jmg has tip frames, set tip_frames_ to jmg tip frames
        // consider removing these lines as they might be unnecessary
        // as tip_frames_ is set by the call to storeValues above
        auto jmg_tips = std::vector<std::string>{};
        jmg_->getEndEffectorTips(jmg_tips);
        if (!jmg_tips.empty()) tip_frames_ = jmg_tips;
        ///////////////////////////////////////////////////////////

        // link_names are the same as tip frames
        // TODO: why do we need to set this
        link_names_ = tip_frames_;

        // Create our internal Robot object from the robot model
        robot_ = Robot::from(robot_model_);

        // Calculate internal state used in IK
        tip_link_indexes_ = get_link_indexes(robot_model_, tip_frames_);
        active_variable_indexes_ =  // jmg_->getKinematicsSolverJointBijection();
            get_active_variable_indexes(robot_model_, jmg_, tip_link_indexes_);
        minimal_displacement_factors_ =
            get_minimal_displacement_factors(active_variable_indexes_, robot_);

        return true;
    }

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state, double timeout, std::vector<double> const&,
        std::vector<double>& solution, IKCallbackFn const& solution_callback,
        IKCostFn cost_function, moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = nullptr) const {
        (void)context_state;

        // Read current ROS parameters
        auto params = parameter_listener_->get_params();

        auto robot_state = moveit::core::RobotState(robot_model_);
        robot_state.setToDefaultValues();
        robot_state.setJointGroupPositions(jmg_, ik_seed_state);
        robot_state.update();
        auto const* variables = robot_state.getVariablePositions();
        auto const count = robot_state.getVariableCount();
        auto const initial_guess = std::vector<double>(variables, variables + count);

        auto const goal_frames = transform_poses_to_frames(robot_state, ik_poses, getBaseFrame());
        auto const frame_tests = make_frame_tests(goal_frames, params.twist_threshold);
        auto const active_initial_guess = ik_seed_state;

        auto const pose_cost_functions =
            make_pose_cost_functions(goal_frames, params.rotation_scale);

        // Create goals
        auto goals = std::vector<Goal>{};
        if (params.center_joints_weight > 0.0) {
            goals.push_back(Goal{make_center_joints_cost_fn(robot_, active_variable_indexes_,
                                                            minimal_displacement_factors_),
                                 params.center_joints_weight});
        }
        if (params.avoid_joint_limits_weight > 0.0) {
            goals.push_back(Goal{make_avoid_joint_limits_cost_fn(robot_, active_variable_indexes_,
                                                                 minimal_displacement_factors_),
                                 params.avoid_joint_limits_weight});
        }
        if (params.minimal_displacement_weight > 0.0) {
            goals.push_back(Goal{
                make_minimal_displacement_cost_fn(ik_seed_state, minimal_displacement_factors_),
                params.minimal_displacement_weight});
        }
        if (cost_function) {
            for (auto const& pose : ik_poses) {
                goals.push_back(Goal{
                    make_ik_cost_fn(pose, cost_function, robot_model_, jmg_, ik_seed_state), 1.0});
            }
        }

        auto const fk_fn = make_fk_fn(robot_model_, jmg_, tip_link_indexes_);
        auto const solution_fn =
            make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk_fn);
        auto const fitness_fn = make_fitness_fn(pose_cost_functions, goals, fk_fn);

        auto const maybe_solution = ik_search(ik_seed_state, robot_, active_variable_indexes_,
                                              fitness_fn, solution_fn, timeout);

        if (!maybe_solution.has_value() && !options.return_approximate_solution) {
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        // wrap angles
        solution = maybe_solution.value();
        jmg_->enforcePositionBounds(solution.data());

        // callback?
        if (solution_callback) {
            // run callback
            solution_callback(ik_poses.front(), solution, error_code);

            // return success if callback has accepted the solution
            return error_code.val == error_code.SUCCESS;
        }

        // return success
        error_code.val = error_code.SUCCESS;
        return true;
    }

    virtual std::vector<std::string> const& getJointNames() const { return joint_names_; }

    virtual std::vector<std::string> const& getLinkNames() const { return link_names_; }

    virtual bool getPositionFK(std::vector<std::string> const&, std::vector<double> const&,
                               std::vector<geometry_msgs::msg::Pose>&) const {
        return false;
    }

    virtual bool getPositionIK(geometry_msgs::msg::Pose const&, std::vector<double> const&,
                               std::vector<double>&, moveit_msgs::msg::MoveItErrorCodes&,
                               kinematics::KinematicsQueryOptions const&) const {
        return false;
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state, double timeout,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state,
                                timeout, std::vector<double>(), solution, IKCallbackFn(),
                                error_code, options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state, double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state,
                                timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                                options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state, double timeout,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state,
                                timeout, std::vector<double>(), solution, solution_callback,
                                error_code, options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state, double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose}, ik_seed_state,
                                timeout, consistency_limits, solution, solution_callback,
                                error_code, options);
    }

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state, double timeout,
        std::vector<double> const& consistency_limits, std::vector<double>& solution,
        IKCallbackFn const& solution_callback, moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = NULL) const {
        return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution,
                                solution_callback, IKCostFn(), error_code, options, context_state);
    }
};

}  // namespace gd_ik

PLUGINLIB_EXPORT_CLASS(gd_ik::GDIKPlugin, kinematics::KinematicsBase);
