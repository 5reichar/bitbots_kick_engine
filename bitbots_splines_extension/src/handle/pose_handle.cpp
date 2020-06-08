#include "handle/pose_handle.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace bitbots_splines {


PoseHandle::PoseHandle(std::shared_ptr<Curve> x, std::shared_ptr<Curve> y, std::shared_ptr<Curve> z,
                       std::shared_ptr<Curve> roll, std::shared_ptr<Curve> pitch, std::shared_ptr<Curve> yaw)
                            :PositionHandle(x, y, z), roll_(std::move(roll)), pitch_(std::move(pitch)), yaw_(std::move(yaw)) {
}

tf2::Transform PoseHandle::get_tf_transform(double time) {
  tf2::Transform trans;
  trans.setOrigin(get_position(time));
  trans.setRotation(get_orientation(time));
  return trans;
}

geometry_msgs::Pose PoseHandle::get_geometry_msg_pose(double time) {
  geometry_msgs::Pose msg;
  msg.position = get_geometry_msg_position(time);
  msg.orientation = get_geometry_msg_orientation(time);
  return msg;
}

geometry_msgs::Quaternion PoseHandle::get_geometry_msg_orientation(double time) {
  geometry_msgs::Quaternion msg;
  tf2::convert(get_orientation(time), msg);
  return msg;
}

tf2::Vector3 PoseHandle::get_euler_angles(double time) {
  tf2::Vector3 pos;
  pos[0] = roll_->position(time);
  pos[1] = pitch_->position(time);
  pos[2] = yaw_->position(time);
  return pos;
}
tf2::Vector3 PoseHandle::get_euler_velocity(double time) {
  tf2::Vector3 vel;
  vel[0] = roll_->velocity(time);
  vel[1] = pitch_->velocity(time);
  vel[2] = yaw_->velocity(time);
  return vel;
}
tf2::Vector3 PoseHandle::get_euler_acceleration(double time) {
  tf2::Vector3 acc;
  acc[0] = roll_->acceleration(time);
  acc[1] = pitch_->acceleration(time);
  acc[2] = yaw_->acceleration(time);
  return acc;
}

tf2::Quaternion PoseHandle::get_orientation(double time) {
  tf2::Quaternion quat;
  tf2::Vector3 rpy = get_euler_angles(time);
  quat.setRPY(rpy[0], rpy[1], rpy[2]);
  quat.normalize();
  return quat;
}

std::shared_ptr<Curve> PoseHandle::roll() {
  return roll_;
}

std::shared_ptr<Curve> PoseHandle::pitch() {
  return pitch_;
}

std::shared_ptr<Curve> PoseHandle::yaw() {
  return yaw_;
}

std::string PoseHandle::get_debug_string() {
  std::string output;
  output += "x:\n" + x_->get_debug_string() + "\n";
  output += "y:\n" + y_->get_debug_string() + "\n";
  output += "z:\n" + z_->get_debug_string() + "\n";
  output += "roll:\n" + roll_->get_debug_string() + "\n";
  output += "pitch:\n" + pitch_->get_debug_string() + "\n";
  output += "yaw:\n" + yaw_->get_debug_string() + "\n";
  return output;
}

}