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

std::string to_string(Frame const& self);
KDL::Frame to_KDL(Frame const& self);
tf2::Vector3 multiply(tf2::Quaternion const& q, tf2::Vector3 const& v);
tf2::Quaternion multiply(tf2::Quaternion const& p, tf2::Quaternion const& q);
Frame concat(Frame const& a, Frame const& b);
Frame concat(Frame const& a, Frame const& b, Frame const& c);
tf2::Quaternion invert(tf2::Quaternion const& q);
Frame invert(Frame const& self);
Frame change(Frame const& a, Frame const& b, Frame const& c);
Frame operator*(Frame const& a, Frame const& b);
Frame& operator*=(Frame& self, Frame const& b);
tf2::Quaternion normalize(tf2::Quaternion const& q);
KDL::Twist frame_twist(Frame const& a, Frame const& b);

}  // namespace gd_ik
