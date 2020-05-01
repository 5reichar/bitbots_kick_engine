#ifndef POSITION_HANDLE_H
#define POSITION_HANDLE_H

#include "spline/curve.h"
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Point.h>
#include <memory>

namespace bitbots_splines {

class PositionHandle {
 public:
    PositionHandle(std::shared_ptr<Curve> x, std::shared_ptr<Curve> y, std::shared_ptr<Curve> z);

    geometry_msgs::Point get_geometry_msg_position(double time);

    tf2::Vector3 get_position(double time);
    tf2::Vector3 get_velocity(double time);
    tf2::Vector3 get_acceleration(double time);

    std::shared_ptr<Curve> x();
    std::shared_ptr<Curve> y();
    std::shared_ptr<Curve> z();

protected:
    std::shared_ptr<Curve> x_;
    std::shared_ptr<Curve> y_;
    std::shared_ptr<Curve> z_;

};
}
#endif //POSITION_HANDLE_H
