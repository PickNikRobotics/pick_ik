#include "gd_ik/problem.hpp"

#include <fmt/format.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

#include <stdexcept>
#include <string>

namespace gd_ik {

auto add_tip_link(Problem& self, moveit::core::LinkModel const& link_model)
    -> size_t {
  if (self.link_tip_indices[link_model.getLinkIndex()] < 0) {
    self.link_tip_indices[link_model.getLinkIndex()] =
        self.tip_link_indices.size();
    self.tip_link_indices.push_back(link_model.getLinkIndex());
  }
  return self.link_tip_indices[link_model.getLinkIndex()];
}

auto add_active_variable(
    Problem& self,
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    moveit::core::JointModelGroup const* jmg, std::string const& name)
    -> size_t {
  for (size_t i = 0; i < self.active_variable_indices.size(); ++i) {
    auto index = self.active_variable_indices.at(i);
    if (name == robot_model->getVariableNames().at(index)) {
      return i;
    }
  }
  for (auto const& joint_name : jmg->getVariableNames()) {
    if (name == joint_name) {
      self.active_variable_indices.push_back(
          robot_model->getVariableIndex(joint_name));
      return self.active_variable_indices.size() - 1;
    }
  }

  throw std::invalid_argument(fmt::format("joint not found: {}", name));
}

auto Problem::from(
    std::shared_ptr<moveit::core::RobotModel const> const& robot_model,
    moveit::core::JointModelGroup const* jmg, std::vector<Goal> const& goals,
    Robot const& robot) -> Problem {
  Problem self;

  // Initialize vectors
  self.link_tip_indices.resize(robot_model->getLinkModelCount(), -1);
  self.joint_usage.resize(robot_model->getJointModelCount(), 0);

  self.goals = goals;
  for (auto const& goal : self.goals) {
    for (auto const& name : goal.link_names) {
      auto const* link_model = robot_model->getLinkModel(name);
      if (!link_model) {
        throw std::invalid_argument(fmt::format("link not found: {}", name));
      }
      add_tip_link(self, *link_model);
    }
  }

  // update active variables from active subtree
  for (auto tip_index : self.tip_link_indices) {
    for (auto* link_model = robot_model->getLinkModels()[tip_index]; link_model;
         link_model = link_model->getParentLinkModel()) {
      self.joint_usage[link_model->getParentJointModel()->getJointIndex()] = 1;
    }
  }

  for (auto const* joint_model : jmg->getActiveJointModels()) {
    if (self.joint_usage[joint_model->getJointIndex()] &&
        !joint_model->getMimic()) {
      for (auto& n : joint_model->getVariableNames()) {
        add_active_variable(self, robot_model, jmg, n);
      }
    }
  }

  // init weights for minimal displacement goals
  self.minimal_displacement_factors.resize(self.active_variable_indices.size());
  if (double s = std::accumulate(self.active_variable_indices.cbegin(),
                                 self.active_variable_indices.cend(), 0.0,
                                 [&robot](auto sum, auto ivar) {
                                   return sum +
                                          get_max_velocity_rcp(robot, ivar);
                                 });
      s > 0) {
    std::transform(self.active_variable_indices.cbegin(),
                   self.active_variable_indices.cend(),
                   self.minimal_displacement_factors.begin(),
                   [&robot, s](auto ivar) {
                     return get_max_velocity_rcp(robot, ivar) / s;
                   });
  } else {
    std::fill(self.minimal_displacement_factors.begin(),
              self.minimal_displacement_factors.end(),
              1.0 / self.active_variable_indices.size());
  }

  return self;
}

}  // namespace gd_ik
