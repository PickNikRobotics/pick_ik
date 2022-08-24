#include <gd_ik/problem.hpp>

#include <fmt/format.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace gd_ik {

auto get_active_variable_indexes(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    moveit::core::JointModelGroup const* jmg,
    std::vector<size_t> const& tip_link_indexes) -> std::vector<size_t> {
  // Walk the tree of links starting at each tip towards the parent
  // The parent joint of each of these links are the ones we are using
  auto joint_usage = std::vector<int>{};
  joint_usage.resize(robot_model->getJointModelCount(), 0);
  for (auto tip_index : tip_link_indexes) {
    for (auto const* link_model = robot_model->getLinkModels().at(tip_index);
         link_model != nullptr; link_model = link_model->getParentLinkModel()) {
      auto const* joint_model = link_model->getParentJointModel();
      auto const joint_index = joint_model->getJointIndex();
      joint_usage[joint_index] = 1;
    }
  }

  // For each of the active joints in the joint model group
  // If those are in the ones we are using and the joint is not a mimic
  // Then get all the variable names from the joint moodel
  auto active_variable_names = std::set<std::string>{};
  for (auto const* joint_model : jmg->getActiveJointModels()) {
    if (joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic()) {
      for (auto& name : joint_model->getVariableNames()) {
        active_variable_names.insert(name);
      }
    }
  }

  // For each active variable name, add the indexes from that variable to the
  // active variables
  auto active_variable_indexes = std::vector<size_t>{};
  for (auto const& name : active_variable_names) {
    active_variable_indexes.push_back(robot_model->getVariableIndex(name));
  }

  return active_variable_indexes;
}

auto get_minimal_displacement_factors(
    std::vector<size_t> const& active_variable_indexes, Robot const& robot)
    -> std::vector<double> {
  auto minimal_displacement_factors = std::vector<double>{};
  minimal_displacement_factors.resize(active_variable_indexes.size());
  if (double s = std::accumulate(
          active_variable_indexes.cbegin(), active_variable_indexes.cend(), 0.0,
          [&robot](auto sum, auto ivar) {
            return sum + get_max_velocity_rcp(robot, ivar);
          });
      s > 0) {
    std::transform(
        active_variable_indexes.cbegin(), active_variable_indexes.cend(),
        minimal_displacement_factors.begin(), [&robot, s](auto ivar) {
          return get_max_velocity_rcp(robot, ivar) / s;
        });
  } else {
    std::fill(minimal_displacement_factors.begin(),
              minimal_displacement_factors.end(),
              1.0 / active_variable_indexes.size());
  }
  return minimal_displacement_factors;
}

auto Problem::from(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    moveit::core::JointModelGroup const* jmg, std::vector<Goal> const& goals,
    Robot const& robot) -> Problem {
  Problem self;

  self.active_variable_indexes =
      get_active_variable_indexes(robot_model, jmg, self.tip_link_indexes);
  self.minimal_displacement_factors =
      get_minimal_displacement_factors(self.active_variable_indexes, robot);

  return self;
}

}  // namespace gd_ik
