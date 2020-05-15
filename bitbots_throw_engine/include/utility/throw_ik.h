#ifndef THROW_IK_H
#define THROW_IK_H

#include "throws/throw_curves/throw_curve.h"
#include "bitbots_splines/abstract_ik.h"

class ThrowIK : public bitbots_splines::AbstractIK
{
public:
  ThrowIK();
  ~ThrowIK();

  bitbots_splines::JointGoals calculate(std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals) override;
  void init(moveit::core::RobotModelPtr kinematic_model) override;
  void reset() override;
  
  void set_bio_ik_timeout(double timeout);

private:
  robot_state::RobotStatePtr goal_state_;
  const moveit::core::JointModelGroup * arms_joints_group_;

  double bio_ik_timeout_;
};

#endif