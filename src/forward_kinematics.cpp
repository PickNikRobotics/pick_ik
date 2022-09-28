#include <pick_ik/forward_kinematics.hpp>

#include <algorithm>
#include <cassert>
#include <iterator>
#include <memory>
#include <moveit/robot_model/robot_model.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>

namespace pick_ik {

auto make_joint_axes(std::shared_ptr<moveit::core::RobotModel const> const& model)
    -> std::vector<tf2::Vector3> {
    auto joint_axes = std::vector<tf2::Vector3>{};
    joint_axes.resize(model->getJointModelCount());
    for (size_t i = 0; i < joint_axes.size(); ++i) {
        auto const* joint_model = model->getJointModel(i);
        if (auto const* j = dynamic_cast<moveit::core::RevoluteJointModel const*>(joint_model)) {
            joint_axes[i] = tf2::Vector3(j->getAxis().x(), j->getAxis().y(), j->getAxis().z());
        }
        if (auto const* j = dynamic_cast<moveit::core::PrismaticJointModel const*>(joint_model)) {
            joint_axes[i] = tf2::Vector3(j->getAxis().x(), j->getAxis().y(), j->getAxis().z());
        }
    }
    return joint_axes;
}

auto make_link_frames(std::shared_ptr<moveit::core::RobotModel const> const& model)
    -> std::vector<Eigen::Isometry3d> {
    std::vector<Eigen::Isometry3d> link_frames;
    std::transform(model->getLinkModels().cbegin(),
                   model->getLinkModels().cend(),
                   link_frames.begin(),
                   [](auto* link_model) { return link_model->getJointOriginTransform(); });
    return link_frames;
}

auto get_frame(moveit::core::JointModel const& joint_model,
               std::vector<double> const& variables,
               std::vector<tf2::Vector3> const& joint_axes) -> Eigen::Isometry3d {
    auto const type = joint_model.getType();
    size_t const index = joint_model.getJointIndex();

    switch (type) {
        case moveit::core::JointModel::FIXED:
            return Eigen::Isometry3d::Identity();
        case moveit::core::JointModel::REVOLUTE: {
            auto const axis = joint_axes.at(index);
            auto const v = variables.at(joint_model.getFirstVariableIndex());
            auto const half_angle = v * 0.5;
            auto const fcos = cos(half_angle);
            auto const fsin = sin(half_angle);

            return Eigen::Isometry3d(
                Eigen::Quaterniond(fcos, axis.x() * fsin, axis.y() * fsin, axis.z() * fsin));
        }
        case moveit::core::JointModel::PRISMATIC: {
            auto const axis = joint_axes.at(index);
            auto const v = variables.at(joint_model.getFirstVariableIndex());
            return Eigen::Isometry3d(
                Eigen::Translation3d(axis.x() * v, axis.y() * v, axis.z() * v));
        }
        case moveit::core::JointModel::FLOATING: {
            assert(joint_model.getFirstVariableIndex() + 6 >= variables.size());
            auto const* vv = variables.data() + joint_model.getFirstVariableIndex();
            // Note that Eigen uses (w, x, y, z) for quaternion notation.
            return Eigen::Translation3d(vv[0], vv[1], vv[2]) *
                   Eigen::Quaterniond(vv[6], vv[3], vv[4], vv[5]);
        }
        case moveit::core::JointModel::PLANAR:
        case moveit::core::JointModel::UNKNOWN:
            break;
    }

    auto const* joint_variables = variables.data() + joint_model.getFirstVariableIndex();
    Eigen::Isometry3d joint_transform;
    joint_model.computeTransform(joint_variables, joint_transform);
    return joint_transform;
}

auto get_frame(moveit::core::LinkModel const& link_model,
               std::vector<Eigen::Isometry3d> const& link_frames) -> Eigen::Isometry3d {
    return link_frames.at(link_model.getLinkIndex());
}

auto has_joint_moved(moveit::core::JointModel const& joint_model,
                     std::vector<double> const& cached_variables,
                     std::vector<double> const& variables) -> bool {
    size_t const i0 = joint_model.getFirstVariableIndex();
    size_t const cnt = joint_model.getVariableCount();
    if (cnt == 0) return true;
    if (cnt == 1) return !(variables.at(i0) == cached_variables.at(i0));
    for (size_t i = i0; i < i0 + cnt; ++i) {
        if (!(variables.at(i) == cached_variables.at(i))) {
            return true;
        }
    }
    return false;
}

auto get_frame(CachedJointFrames& cache,
               moveit::core::JointModel const& joint_model,
               std::vector<double> const& variables,
               std::vector<tf2::Vector3> const& joint_axes) -> Eigen::Isometry3d {
    size_t const index = joint_model.getJointIndex();

    if (!has_joint_moved(joint_model, cache.variables, variables)) {
        return cache.frames.at(index);
    }

    // Update frame in cache
    cache.frames.at(index) = get_frame(joint_model, variables, joint_axes);

    // Update variables in cache
    auto const cnt = joint_model.getVariableCount();
    auto const i0 = joint_model.getFirstVariableIndex();
    if (cnt > 0) {
        for (size_t i = i0; i < i0 + cnt; ++i) {
            cache.variables.at(i) = variables.at(i);
        }
    }

    return cache.frames.at(index);
}

}  // namespace pick_ik
