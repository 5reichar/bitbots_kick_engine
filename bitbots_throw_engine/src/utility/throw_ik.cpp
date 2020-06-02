#include "utility/throw_ik.h"

namespace bitbots_throw{
  ThrowIK::ThrowIK(){
    bio_ik_timeout_ = 0.01;
  }

  ThrowIK::~ThrowIK(){
    if (all_joints_group_) delete all_joints_group_;
  }

  void ThrowIK::init(moveit::core::RobotModelPtr kinematic_model){
    all_joints_group_ = kinematic_model->getJointModelGroup("All");
    goal_state_.reset(new robot_state::RobotState(kinematic_model));
    goal_state_->setToDefaultValues();
    reset();
  }

  bitbots_splines::JointGoals ThrowIK::calculate(const std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals){
    if (!goal_state_->setFromIK(all_joints_group_,
                                EigenSTL::vector_Isometry3d(),
                                std::vector<std::string>(),
                                bio_ik_timeout_,
                                moveit::core::GroupStateValidityCallbackFn(),
                                *ik_goals)){
      throw std::runtime_error("ThrowIK::calculate: Calculation failed");
    }

    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = all_joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(all_joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  }

  void ThrowIK::reset(){
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    std::vector<std::string> names_vec = {"LElbow", "LShoulderPitch", "LShoulderRoll", "RElbow", "RShoulderPitch", "RShoulderRoll"};
    std::vector<double> pos_vec = {0.7, -1.0, -0.4, -0.7, 1.0, 0.4};
    for (int i = 0; i < names_vec.size(); i++){
      // besides its name, this method only changes a single joint position...
      goal_state_->setJointPositions(names_vec[i], &pos_vec[i]);
    }
  }

  void ThrowIK::set_bio_ik_timeout(double timeout){
    bio_ik_timeout_ = timeout;
  }
} //bitbots_throw