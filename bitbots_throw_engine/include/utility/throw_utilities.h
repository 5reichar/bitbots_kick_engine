#ifndef THROW_UTILITIES_H
#define THROW_UTILITIES_H

#include <string>
#include "parameter/struct3d.h"
#include <geometry_msgs/Quaternion.h>

enum class CurvePurpose
{
    trunk,
    left_foot,
    right_foot,
    left_hand,
    right_hand,
    arms,
    left_elbow,
    left_shoulder_pitch,
    left_shoulder_roll,
    right_elbow,
    right_shoulder_pitch,
    right_shoulder_roll
};

struct ThrowRequest
{
    Struct3d ball_position_;
    Struct3d goal_position_;
};

struct ThrowResponse
{
  tf2::Transform support_foot_to_flying_foot;
  tf2::Transform support_foot_to_trunk;

  // additional information for visualization
  double phase;
  double traj_time;
};

static std::string get_joint_name(CurvePurpose joint)
{
  std::string joint_name = "";

  switch (joint)
  {
    case arms:
      joint_name = "Arms";
      break;
    case left_elbow:
      joint_name = "LElbow";
      break;
    case left_shoulder_pitch:
      joint_name = "LShoulderPitch";
      break;
    case left_shoulder_roll:
      joint_name = "LShoulderRoll";
      break;
    case right_elbow:
      joint_name = "RElbow";
      break;
    case right_shoulder_pitch:
      joint_name = "RShoulderPitch";
      break;
    case right_shoulder_roll:
      joint_name = "RShoulderRoll";
      break;
    default:
      break;
  }

  return joint_name;
};

#endif //THROW_UTILITIES_H