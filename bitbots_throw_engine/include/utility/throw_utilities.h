#ifndef THROW_UTILITIES_H
#define THROW_UTILITIES_H

#include <string>
#include "parameter/struct3d.h"
#include "tf2/LinearMath/Transform.h"

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
  tf2::Transform support_foot_to_left_hand_;
  tf2::Transform support_foot_to_right_hand_;
  tf2::Transform support_foot_to_trunk_;

  // additional information for visualization
  double phase_;
  double traj_time_;
};

static std::string get_joint_name(CurvePurpose joint)
{
  std::string joint_name = "";

  switch (joint)
  {
    case CurvePurpose::arms:
      joint_name = "Arms";
      break;
    case CurvePurpose::left_elbow:
      joint_name = "LElbow";
      break;
    case CurvePurpose::left_shoulder_pitch:
      joint_name = "LShoulderPitch";
      break;
    case CurvePurpose::left_shoulder_roll:
      joint_name = "LShoulderRoll";
      break;
    case CurvePurpose::right_elbow:
      joint_name = "RElbow";
      break;
    case CurvePurpose::right_shoulder_pitch:
      joint_name = "RShoulderPitch";
      break;
    case CurvePurpose::right_shoulder_roll:
      joint_name = "RShoulderRoll";
      break;
    default:
      break;
  }

  return joint_name;
};

static double calculate_distace(Struct3d point)
{
    return sqrt(pow(point.x_, 2) + pow(point.y_, 2));
}

#endif //THROW_UTILITIES_H