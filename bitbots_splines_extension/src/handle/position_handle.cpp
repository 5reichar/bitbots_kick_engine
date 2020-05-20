#include "handle/position_handle.h"

namespace bitbots_splines {

PositionHandle::PositionHandle(std::shared_ptr<Curve> x, std::shared_ptr<Curve> y, std::shared_ptr<Curve> z) 
    : x_(std::move(x)), y_(std::move(y)), z_(std::move(z)) {
}

geometry_msgs::Point PositionHandle::get_geometry_msg_position(double time) {
  geometry_msgs::Point msg;
  tf2::Vector3 tf_vec = get_position(time);
  msg.x = tf_vec.x();
  msg.y = tf_vec.y();
  msg.z = tf_vec.z();
  return msg;
}

tf2::Vector3 PositionHandle::get_position(double time) {
  tf2::Vector3 pos;
  pos[0] = x_->position(time);
  pos[1] = y_->position(time);
  pos[2] = z_->position(time);
  return pos;
}

tf2::Vector3 PositionHandle::get_velocity(double time) {
  tf2::Vector3 vel;
  vel[0] = x_->velocity(time);
  vel[1] = y_->velocity(time);
  vel[2] = z_->velocity(time);
  return vel;
}
tf2::Vector3 PositionHandle::get_acceleration(double time) {
  tf2::Vector3 acc;
  acc[0] = x_->acceleration(time);
  acc[1] = y_->acceleration(time);
  acc[2] = z_->acceleration(time);
  return acc;
}

std::shared_ptr<Curve> PositionHandle::x() {
  return std::move(x_);
}

std::shared_ptr<Curve> PositionHandle::y() {
  return std::move(y_);
}

std::shared_ptr<Curve> PositionHandle::z() {
  return std::move(z_);
}

}
