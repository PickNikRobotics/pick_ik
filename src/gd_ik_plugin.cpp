#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/joint_model_group.h>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "gd_ik/robot.hpp"

namespace gd_ik {
namespace {
auto const LOGGER = rclcpp::get_logger("gd_ik");
}

class GDIKPlugin : public kinematics::KinematicsBase {
  moveit::core::JointModelGroup const* jmg_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;
  Robot robot_;

  // std::unique_ptr<IKParallel> ik;
  // std::vector<double> state_;
  // std::vector<double> temp_;
  // std::unique_ptr<moveit::core::RobotState> temp_state_;
  // std::vector<Frame> tipFrames_;

  rclcpp::Node::SharedPtr node_;

 public:
  bool searchPositionIK(
      std::vector<geometry_msgs::msg::Pose> const& ik_poses,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double> const& consistency_limits,
      std::vector<double>& solution, IKCallbackFn const& solution_callback,
      IKCostFn cost_function, moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions(),
      moveit::core::RobotState const* context_state = nullptr) const override {
    // TODO implement
    return false;
  }

  // kinematics::KinematicsBase ////////////////////////////////////////////////
  bool initialize(rclcpp::Node::SharedPtr const& node,
                  moveit::core::RobotModel const& robot_model,
                  std::string const& group_name, std::string const& base_frame,
                  std::vector<std::string> const& tip_frames,
                  double search_discretization) override {
    node_ = node;

    // Initialize internal state of base class KinematicsBase
    // Creates these internal state variables:
    // robot_model_ <- shared_ptr to RobotModel
    // robot_description_ <- empty string
    // group_name_ <- group_name string
    // base_frame_ <- base_frame without leading /
    // tip_frames_ <- tip_frames without leading /
    // redundant_joint_discretization_ <- vector initialized with
    // search_discretization
    storeValues(robot_model, group_name, base_frame, tip_frames,
                search_discretization);

    // Initialize internal state
    jmg_ = robot_model_->getJointModelGroup(group_name);
    if (!jmg_) {
      RCLCPP_ERROR(LOGGER, "failed to get joint model group %s",
                   group_name.c_str());
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
    auto jmg_tips = std::vector<std::string>{};
    jmg_->getEndEffectorTips(jmg_tips);
    if (!jmg_tips.empty()) tip_frames_ = jmg_tips;

    // link_names are the same as tip frames
    link_names_ = tip_frames_;

    // Create our internal Robot object from the robot model
    robot_ = Robot::from(robot_model_);

    return false;
  }

  virtual std::vector<std::string> const& getJointNames() const {
    return joint_names_;
  }

  virtual std::vector<std::string> const& getLinkNames() const {
    return link_names_;
  }

  virtual bool getPositionFK(
      std::vector<std::string> const& link_names,
      std::vector<double> const& joint_angles,
      std::vector<geometry_msgs::msg::Pose>& poses) const {
    return false;
  }

  virtual bool getPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                             std::vector<double> const& ik_seed_state,
                             std::vector<double>& solution,
                             moveit_msgs::msg::MoveItErrorCodes& error_code,
                             kinematics::KinematicsQueryOptions const& options =
                                 kinematics::KinematicsQueryOptions()) const {
    return false;
  }

  virtual bool searchPositionIK(
      geometry_msgs::msg::Pose const& ik_pose,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions()) const {
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, IKCallbackFn(), error_code, options);
  }

  virtual bool searchPositionIK(
      geometry_msgs::msg::Pose const& ik_pose,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double> const& consistency_limits,
      std::vector<double>& solution,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions()) const {
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, IKCallbackFn(), error_code, options);
  }

  virtual bool searchPositionIK(
      geometry_msgs::msg::Pose const& ik_pose,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double>& solution, IKCallbackFn const& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions()) const {
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, solution_callback, error_code, options);
  }

  virtual bool searchPositionIK(
      geometry_msgs::msg::Pose const& ik_pose,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double> const& consistency_limits,
      std::vector<double>& solution, IKCallbackFn const& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions()) const {
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  virtual bool searchPositionIK(
      std::vector<geometry_msgs::msg::Pose> const& ik_poses,
      std::vector<double> const& ik_seed_state, double timeout,
      std::vector<double> const& consistency_limits,
      std::vector<double>& solution, IKCallbackFn const& solution_callback,
      moveit_msgs::msg::MoveItErrorCodes& error_code,
      kinematics::KinematicsQueryOptions const& options =
          kinematics::KinematicsQueryOptions(),
      moveit::core::RobotState const* context_state = NULL) const {
    // TODO implement
    return false;
  }
};

}  // namespace gd_ik

PLUGINLIB_EXPORT_CLASS(gd_ik::GDIKPlugin, kinematics::KinematicsBase);
