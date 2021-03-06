#ifndef BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_
#define BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_

#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/Marker.h>
#include "handle/pose_handle.h"

namespace bitbots_splines {

class AbstractVisualizer {
 protected:
  /**
   * Utility function to create a visualization marker for a position with default properties.
   * @param position The position of the marker
   * @param frame The frame in which the position is given
   * @return The visualization marker
   */
  visualization_msgs::Marker getMarker(const tf2::Vector3 &position, const std::string &frame) {
    visualization_msgs::Marker marker;

    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.color.a = 1;

    return marker;
  }

  /**
   * Utility function to create a visualization marker for a trajectory with default properties.
   * @param pose A PoseHandle containing the pose to be displayed
   * @param frame The frame in which the pose are given
   * @param smoothness The smoothness of the pose
   * @return The visualization marker
   */
  visualization_msgs::Marker getPath(PoseHandle &pose,
                                     const std::string &frame,
                                     const double smoothness) {
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.lifetime = ros::Duration(1000);
    marker.frame_locked = false;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.01;
    marker.color.a = 1;

    // Take length of spline, assuming values at the beginning and end are set for x
    double first_time = pose.x()->min();
    double last_time = pose.x()->max();

    for (double i = first_time; i <= last_time; i += (last_time - first_time) / smoothness) {
      geometry_msgs::Point point;
      point.x = pose.x()->position(i);
      point.y = pose.y()->position(i);
      point.z = pose.z()->position(i);

      marker.points.push_back(point);
    }

    return marker;
  }
};
}

#endif //BITBOTS_SPLINES_INCLUDE_BITBOTS_SPLINES_ABSTRACT_VISUALIZER_H_
