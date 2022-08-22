#include "gd_ik/frame.hpp"

#include <fmt/format.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Dense>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_kdl/tf2_kdl.hpp>

namespace gd_ik {

Frame Frame::from(const KDL::Frame& kdl) {
  Frame frame;
  frame.pos = tf2::Vector3(kdl.p.x(), kdl.p.y(), kdl.p.z());
  double qx, qy, qz, qw;
  kdl.M.GetQuaternion(qx, qy, qz, qw);
  frame.rot = tf2::Quaternion(qx, qy, qz, qw);
  return frame;
}

Frame Frame::from(geometry_msgs::msg::Pose const& msg) {
  Frame frame;
  frame.pos = tf2::Vector3(msg.position.x, msg.position.y, msg.position.z);
  tf2::fromMsg(msg.orientation, frame.rot);
  return frame;
}

Frame Frame::from(Eigen::Isometry3d const& f) {
  Frame frame;
  frame.pos = tf2::Vector3(f.translation().x(), f.translation().y(),
                           f.translation().z());
  Eigen::Quaterniond q(f.rotation());
  frame.rot = tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
  return frame;
}

Frame Frame::identity() {
  return Frame{
      tf2::Vector3(0, 0, 0),
      tf2::Quaternion(0, 0, 0, 1),
  };
}

std::string to_string(Frame const& self) {
  return fmt::format("(pos: [{}, {}, {}], rot: [{}, {}, {}, {}])", self.pos.x(),
                     self.pos.y(), self.pos.z(), self.rot.x(), self.rot.y(),
                     self.rot.z(), self.rot.w());
}

KDL::Frame to_KDL(Frame const& self) {
  KDL::Frame kdl_frame;
  kdl_frame.p.x(self.pos.x());
  kdl_frame.p.y(self.pos.y());
  kdl_frame.p.z(self.pos.z());
  kdl_frame.M = KDL::Rotation::Quaternion(self.rot.x(), self.rot.y(),
                                          self.rot.z(), self.rot.w());
  return kdl_frame;
}

tf2::Vector3 multiply(tf2::Quaternion const& q, tf2::Vector3 const& v) {
  double const v_x = v.x();
  double const v_y = v.y();
  double const v_z = v.z();

  double const q_x = q.x();
  double const q_y = q.y();
  double const q_z = q.z();
  double const q_w = q.w();

  if ((v_x == 0 && v_y == 0 && v_z == 0) ||
      (q_x == 0 && q_y == 0 && q_z == 0 && q_w == 1)) {
    return v;
  }

  double const t_x = q_y * v_z - q_z * v_y;
  double const t_y = q_z * v_x - q_x * v_z;
  double const t_z = q_x * v_y - q_y * v_x;

  double r_x = q_w * t_x + q_y * t_z - q_z * t_y;
  double r_y = q_w * t_y + q_z * t_x - q_x * t_z;
  double r_z = q_w * t_z + q_x * t_y - q_y * t_x;

  r_x += r_x;
  r_y += r_y;
  r_z += r_z;

  r_x += v_x;
  r_y += v_y;
  r_z += v_z;

  tf2::Vector3 ret;
  ret.setX(r_x);
  ret.setY(r_y);
  ret.setZ(r_z);
  return ret;
}

tf2::Quaternion multiply(tf2::Quaternion const& p, tf2::Quaternion const& q) {
  double const p_x = p.x();
  double const p_y = p.y();
  double const p_z = p.z();
  double const p_w = p.w();

  double const q_x = q.x();
  double const q_y = q.y();
  double const q_z = q.z();
  double const q_w = q.w();

  double const r_x = (p_w * q_x + p_x * q_w) + (p_y * q_z - p_z * q_y);
  double const r_y = (p_w * q_y - p_x * q_z) + (p_y * q_w + p_z * q_x);
  double const r_z = (p_w * q_z + p_x * q_y) - (p_y * q_x - p_z * q_w);
  double const r_w = (p_w * q_w - p_x * q_x) - (p_y * q_y + p_z * q_z);

  tf2::Quaternion ret;
  ret.setX(r_x);
  ret.setY(r_y);
  ret.setZ(r_z);
  ret.setW(r_w);
  return ret;
}

Frame concat(Frame const& a, Frame const& b) {
  return Frame{
      a.pos + multiply(a.rot, b.pos),
      multiply(a.rot, b.rot),
  };
}

Frame concat(Frame const& a, Frame const& b, Frame const& c) {
  return concat(concat(a, b), c);
}

tf2::Quaternion invert(tf2::Quaternion const& q) {
  tf2::Quaternion ret;
  ret.setX(-q.x());
  ret.setY(-q.y());
  ret.setZ(-q.z());
  ret.setW(q.w());
  return ret;
}

Frame invert(Frame const& self) {
  auto rot = invert(self.rot);
  auto pos = multiply(rot, -self.pos);
  return Frame{pos, rot};
}

Frame change(Frame const& a, Frame const& b, Frame const& c) {
  return concat(a, invert(b), c);
}

Frame operator*(Frame const& a, Frame const& b) { return concat(a, b); }

Frame& operator*=(Frame& self, Frame const& b) {
  self = self * b;
  return self;
}

tf2::Quaternion normalize(tf2::Quaternion const& q) {
  tf2::Quaternion ret;
  double f = (3.0 - q.length2()) * 0.5;
  ret.setX(q.x() * f);
  ret.setY(q.y() * f);
  ret.setZ(q.z() * f);
  ret.setW(q.w() * f);
  return ret;
}

KDL::Twist frame_twist(Frame const& a, Frame const& b) {
  auto const frame = invert(a) * b;
  KDL::Twist twist;
  twist.vel.x(frame.pos.x());
  twist.vel.y(frame.pos.y());
  twist.vel.z(frame.pos.z());

  double ra = frame.rot.getAngle();
  if (ra > +M_PI) ra -= 2 * M_PI;

  auto const r = frame.rot.getAxis() * ra;
  twist.rot.x(r.x());
  twist.rot.y(r.y());
  twist.rot.z(r.z());

  return twist;
}

}  // namespace gd_ik
