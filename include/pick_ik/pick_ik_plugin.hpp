#pragma once

#include <pick_ik/robot.hpp>

#include <moveit/kinematics_base/kinematics_base.h>

namespace pick_ik {

class ParamListener;

class PickIKPlugin : public kinematics::KinematicsBase {
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<ParamListener> parameter_listener_;
    moveit::core::JointModelGroup const* jmg_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> link_names_;
    std::vector<size_t> tip_link_indices_;
    Robot robot_;

    mutable std::mutex fk_mutex_;

   public:
    virtual bool initialize(rclcpp::Node::SharedPtr const& node,
                            moveit::core::RobotModel const& robot_model,
                            std::string const& group_name,
                            std::string const& base_frame,
                            std::vector<std::string> const& tip_frames,
                            double search_discretization);

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state,
        double timeout,
        std::vector<double> const&,
        std::vector<double>& solution,
        IKCallbackFn const& solution_callback,
        IKCostFn const& cost_function,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = nullptr) const;

    virtual std::vector<std::string> const& getJointNames() const;

    virtual std::vector<std::string> const& getLinkNames() const;

    virtual bool getPositionFK(std::vector<std::string> const&,
                               std::vector<double> const&,
                               std::vector<geometry_msgs::msg::Pose>&) const;

    virtual bool getPositionIK(geometry_msgs::msg::Pose const&,
                               std::vector<double> const&,
                               std::vector<double>&,
                               moveit_msgs::msg::MoveItErrorCodes&,
                               kinematics::KinematicsQueryOptions const&) const;

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(geometry_msgs::msg::Pose const& ik_pose,
                                  std::vector<double> const& ik_seed_state,
                                  double timeout,
                                  std::vector<double> const& consistency_limits,
                                  std::vector<double>& solution,
                                  IKCallbackFn const& solution_callback,
                                  moveit_msgs::msg::MoveItErrorCodes& error_code,
                                  kinematics::KinematicsQueryOptions const& options =
                                      kinematics::KinematicsQueryOptions()) const;

    virtual bool searchPositionIK(
        std::vector<geometry_msgs::msg::Pose> const& ik_poses,
        std::vector<double> const& ik_seed_state,
        double timeout,
        std::vector<double> const& consistency_limits,
        std::vector<double>& solution,
        IKCallbackFn const& solution_callback,
        moveit_msgs::msg::MoveItErrorCodes& error_code,
        kinematics::KinematicsQueryOptions const& options = kinematics::KinematicsQueryOptions(),
        moveit::core::RobotState const* context_state = NULL) const;
};

}  // namespace pick_ik
