#include <pick_ik/fk_moveit.hpp>
#include <pick_ik/goal.hpp>
#include <pick_ik/ik_gradient.hpp>
#include <pick_ik/ik_memetic.hpp>
#include <pick_ik/robot.hpp>

#include <pick_ik_parameters.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_state/robot_state.h>
#include <string>
#include <vector>

namespace pick_ik {
namespace {
auto const LOGGER = rclcpp::get_logger("pick_ik");
}

class PickIKPlugin : public kinematics::KinematicsBase {
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ParamListener> parameter_listener_;
    moveit::core::JointModelGroup const* jmg_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<size_t> tip_link_indices_;
    Robot robot_;

   public:
    virtual bool initialize(rclcpp::Node::SharedPtr const& node,
                            moveit::core::RobotModel const& robot_model,
                            std::string const& group_name,
                            std::string const& base_frame,
                            std::vector<std::string> const& tip_frames,
                            double search_discretization) {
        node_ = node;
        parameter_listener_ = std::make_shared<ParamListener>(
            node,
            std::string("robot_description_kinematics.").append(group_name));

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

        // link_names are the same as tip frames
        // TODO: why do we need to set this
        link_names_ = tip_frames_;

        // Create our internal Robot object from the robot model
        tip_link_indices_ =
            get_link_indices(robot_model_, tip_frames_)
                .or_else([](auto const& error) { throw std::invalid_argument(error); })
                .value();
        robot_ = Robot::from(robot_model_, jmg_, tip_link_indices_);

        return true;
    }

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state,
        double timeout,
        std::vector<double> const&,
        std::vector<double>& solution,
        IKCallbackFn const& solution_callback,
        IKCostFn cost_function,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = nullptr) const {
        (void)context_state;  // not used

        // Read current ROS parameters
        auto params = parameter_listener_->get_params();

        auto const goal_frames = [&]() {
            auto robot_state = moveit::core::RobotState(robot_model_);
            robot_state.setToDefaultValues();
            robot_state.setJointGroupPositions(jmg_, ik_seed_state);
            robot_state.update();
            return transform_poses_to_frames(robot_state, ik_poses, getBaseFrame());
        }();

        // Test functions to determine if we are at our goal frame
        auto const test_rotation = (params.rotation_scale > 0);
        std::optional<double> orientation_threshold = std::nullopt;
        if (test_rotation) {
            orientation_threshold = params.orientation_threshold;
        }
        auto const frame_tests =
            make_frame_tests(goal_frames, params.position_threshold, orientation_threshold);

        // Cost functions used for optimizing towards goal frames
        auto const pose_cost_functions =
            make_pose_cost_functions(goal_frames, params.rotation_scale);

        // forward kinematics function
        auto const fk_fn = make_fk_fn(robot_model_, jmg_, tip_link_indices_);

        // Create goals (weighted cost functions)
        auto goals = std::vector<Goal>{};
        if (params.center_joints_weight > 0.0) {
            goals.push_back(Goal{make_center_joints_cost_fn(robot_), params.center_joints_weight});
        }
        if (params.avoid_joint_limits_weight > 0.0) {
            goals.push_back(
                Goal{make_avoid_joint_limits_cost_fn(robot_), params.avoid_joint_limits_weight});
        }
        if (params.minimal_displacement_weight > 0.0) {
            goals.push_back(Goal{make_minimal_displacement_cost_fn(robot_, ik_seed_state),
                                 params.minimal_displacement_weight});
        }
        if (cost_function) {
            for (auto const& pose : ik_poses) {
                goals.push_back(
                    Goal{make_ik_cost_fn(pose, cost_function, robot_model_, jmg_, ik_seed_state),
                         1.0});
            }
        }

        // test if this is a valid solution
        auto const solution_fn =
            make_is_solution_test_fn(frame_tests, goals, params.cost_threshold, fk_fn);

        // single function used by gradient descent to calculate cost of solution
        auto const cost_fn = make_cost_fn(pose_cost_functions, goals, fk_fn);

        // Search for a solution using either the local or global solver.
        std::optional<std::vector<double>> maybe_solution;
        if (params.mode == "global") {
            MemeticIkParams ik_params;
            ik_params.population_size = static_cast<size_t>(params.memetic_population_size);
            ik_params.elite_size = static_cast<size_t>(params.memetic_elite_size);
            ik_params.wipeout_fitness_tol = params.memetic_wipeout_fitness_tol;
            ik_params.num_threads = static_cast<size_t>(params.memetic_num_threads);
            ik_params.stop_on_first_soln = params.memetic_stop_on_first_solution;
            ik_params.max_generations = static_cast<int>(params.memetic_max_generations);
            ik_params.max_time = timeout;

            ik_params.gd_params.step_size = params.gd_step_size;
            ik_params.gd_params.min_cost_delta = params.gd_min_cost_delta;
            ik_params.gd_params.max_iterations = static_cast<int>(params.memetic_gd_max_iters);
            ik_params.gd_params.max_time = params.memetic_gd_max_time;

            maybe_solution = ik_memetic(ik_seed_state,
                                        robot_,
                                        cost_fn,
                                        solution_fn,
                                        ik_params,
                                        options.return_approximate_solution,
                                        false /* No debug print */);
        } else if (params.mode == "local") {
            GradientIkParams gd_params;
            gd_params.step_size = params.gd_step_size;
            gd_params.min_cost_delta = params.gd_min_cost_delta;
            gd_params.max_time = timeout;
            gd_params.max_iterations = static_cast<int>(params.gd_max_iters);

            maybe_solution = ik_gradient(ik_seed_state,
                                         robot_,
                                         cost_fn,
                                         solution_fn,
                                         gd_params,
                                         options.return_approximate_solution);
        } else {
            RCLCPP_ERROR(LOGGER, "Invalid solver mode: %s", params.mode.c_str());
            return false;
        }

        if (maybe_solution.has_value()) {
            // Set the output parameter solution.
            // Assumes that the angles were already wrapped by the solver.
            error_code.val = error_code.SUCCESS;
            solution = maybe_solution.value();
            if (jmg_->enforcePositionBounds(solution.data())) {
                error_code.val = error_code.NO_IK_SOLUTION;
                solution = ik_seed_state;
            }
        } else {
            error_code.val = error_code.NO_IK_SOLUTION;
            solution = ik_seed_state;
        }

        // If using an approximate solution, check against the maximum allowable pose threshold.
        // If the approximate solution is too far from the goal frame, fall back to the initial
        // state.
        if (options.return_approximate_solution) {
            std::optional<double> approximate_solution_orientation_threshold = std::nullopt;
            if (test_rotation) {
                approximate_solution_orientation_threshold =
                    params.approximate_solution_orientation_threshold;
            }
            auto const approx_frame_tests =
                make_frame_tests(goal_frames,
                                 params.approximate_solution_position_threshold,
                                 approximate_solution_orientation_threshold);
            auto const tip_frames = fk_fn(solution);
            bool approx_solution_valid = true;
            for (size_t i = 0; i < approx_frame_tests.size(); ++i) {
                if (!approx_frame_tests[i](tip_frames[i])) {
                    approx_solution_valid = false;
                    break;
                }
            }
            if (!approx_solution_valid) {
                error_code.val = error_code.NO_IK_SOLUTION;
                solution = ik_seed_state;
            }
        }

        // Execute solution callback only on successful solution.
        auto const found_solution = error_code.val == error_code.SUCCESS;
        if (solution_callback && found_solution) {
            solution_callback(ik_poses.front(), solution, error_code);
        }

        return found_solution;
    }

    virtual std::vector<std::string> const& getJointNames() const { return joint_names_; }

    virtual std::vector<std::string> const& getLinkNames() const { return link_names_; }

    virtual bool getPositionFK(std::vector<std::string> const&,
                               std::vector<double> const&,
                               std::vector<geometry_msgs::msg::Pose>&) const {
        return false;
    }

    virtual bool getPositionIK(geometry_msgs::msg::Pose const&,
                               std::vector<double> const&,
                               std::vector<double>&,
                               moveit_msgs::msg::MoveItErrorCodes&,
                               kinematics::KinematicsQueryOptions const&) const {
        return false;
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                ik_seed_state,
                                timeout,
                                std::vector<double>(),
                                solution,
                                IKCallbackFn(),
                                error_code,
                                options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                IKCallbackFn(),
                                error_code,
                                options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                ik_seed_state,
                                timeout,
                                std::vector<double>(),
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const {
        return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                solution_callback,
                                error_code,
                                options);
    }

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state,
        double timeout,
        std::vector<double> const& consistency_limits,
        std::vector<double>& solution,
        IKCallbackFn const& solution_callback,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = NULL) const {
        return searchPositionIK(ik_poses,
                                ik_seed_state,
                                timeout,
                                consistency_limits,
                                solution,
                                solution_callback,
                                IKCostFn(),
                                error_code,
                                options,
                                context_state);
    }
};

}  // namespace pick_ik

PLUGINLIB_EXPORT_CLASS(pick_ik::PickIKPlugin, kinematics::KinematicsBase);
