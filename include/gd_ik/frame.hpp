#pragma once

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Dense>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <vector>

namespace gd_ik {

struct Frame {
  tf2::Vector3 pos;
  tf2::Quaternion rot;

  static Frame from(KDL::Frame const& kdl);
  static Frame from(geometry_msgs::msg::Pose const& msg);
  static Frame from(Eigen::Isometry3d const& f);
  static Frame identity();
};

auto to_string(Frame const& self) -> std::string;
auto to_KDL(Frame const& self) -> KDL::Frame;
auto multiply(tf2::Quaternion const& q, tf2::Vector3 const& v) -> tf2::Vector3;
auto multiply(tf2::Quaternion const& p, tf2::Quaternion const& q)
    -> tf2::Quaternion;
auto concat(Frame const& a, Frame const& b) -> Frame;
auto concat(Frame const& a, Frame const& b, Frame const& c) -> Frame;
auto invert(tf2::Quaternion const& q) -> tf2::Quaternion;
auto invert(Frame const& self) -> Frame;
auto change(Frame const& a, Frame const& b, Frame const& c) -> Frame;
auto operator*(Frame const& a, Frame const& b) -> Frame;
auto operator*=(Frame& self, Frame const& b) -> Frame&;
auto normalize(tf2::Quaternion const& q) -> tf2::Quaternion;
auto frame_twist(Frame const& a, Frame const& b) -> KDL::Twist;

}  // namespace gd_ik
