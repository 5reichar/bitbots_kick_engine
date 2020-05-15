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
  pos[0] = x_->pos(time);
  pos[1] = y_->pos(time);
  pos[2] = z_->pos(time);
  return pos;
}

tf2::Vector3 PositionHandle::get_velocity(double time) {
  tf2::Vector3 vel;
  vel[0] = x_->vel(time);
  vel[1] = y_->vel(time);
  vel[2] = z_->vel(time);
  return vel;
}
tf2::Vector3 PositionHandle::get_acceleration(double time) {
  tf2::Vector3 acc;
  acc[0] = x_->acc(time);
  acc[1] = y_->acc(time);
  acc[2] = z_->acc(time);
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
