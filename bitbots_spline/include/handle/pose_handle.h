#ifndef BITBOTS_SPLINES_EXTENSION_POSE_HANDLE_H
#define BITBOTS_SPLINES_EXTENSION_POSE_HANDLE_H

#include "handle/position_handle.h"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

namespace bitbots_splines {

class PoseHandle : public PositionHandle{
public:
    PoseHandle(std::shared_ptr<Curve> x, std::shared_ptr<Curve> y, std::shared_ptr<Curve> z,
               std::shared_ptr<Curve> roll, std::shared_ptr<Curve> pitch, std::shared_ptr<Curve> yaw);

    tf2::Transform get_tf_transform(double time);

    geometry_msgs::Pose get_geometry_msg_pose(double time);
    geometry_msgs::Quaternion get_geometry_msg_orientation(double time);

    tf2::Vector3 get_euler_angles(double time);
    tf2::Vector3 get_euler_velocity(double time);
    tf2::Vector3 get_euler_acceleration(double time);

    tf2::Quaternion get_orientation(double time);

    std::string get_debug_string();

    std::shared_ptr<Curve> roll();
    std::shared_ptr<Curve> pitch();
    std::shared_ptr<Curve> yaw();

private:
    std::shared_ptr<Curve> roll_;
    std::shared_ptr<Curve> pitch_;
    std::shared_ptr<Curve> yaw_;

};
}
#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_POSE_SPLINE_H_
