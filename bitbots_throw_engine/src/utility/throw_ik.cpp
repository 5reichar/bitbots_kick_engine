#include "utility/throw_ik.h"

namespace bitbots_throw{
  ThrowIK::ThrowIK(std::string joint_group_name
                  ,std::vector<std::string> joint_names
                  ,std::vector<double> initial_joint_position){
    bio_ik_timeout_ = 0.01;
    joint_group_name_ = joint_group_name;
    joint_names_ = joint_names;
    initial_joint_position_ = initial_joint_position;
    joints_group_ = nullptr;
  }

  ThrowIK::~ThrowIK(){
    delete joints_group_;
  }

  void ThrowIK::init(moveit::core::RobotModelPtr kinematic_model){
    joints_group_ = kinematic_model->getJointModelGroup(joint_group_name_);
    goal_state_.reset(new robot_state::RobotState(kinematic_model));
    goal_state_->setToDefaultValues();
    reset();
  }

  bitbots_splines::JointGoals ThrowIK::calculate(const std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_goals){
    if (!goal_state_->setFromIK(joints_group_
                               ,EigenSTL::vector_Isometry3d()
                               ,std::vector<std::string>()
                               ,bio_ik_timeout_
                               ,moveit::core::GroupStateValidityCallbackFn()
                               ,*ik_goals)){
      throw std::runtime_error("ThrowIK::calculate: Calculation failed");
    }

    /* retrieve joint names and associated positions from  */
    std::vector<std::string> joint_names = joints_group_->getActiveJointModelNames();
    std::vector<double> joint_goals;
    goal_state_->copyJointGroupPositions(joints_group_, joint_goals);

    /* construct result object */
    bitbots_splines::JointGoals result;
    result.first = joint_names;
    result.second = joint_goals;
    return result;
  }

  void ThrowIK::reset(){
    // we have to set some good initial position in the goal state, since we are using a gradient
    // based method. Otherwise, the first step will be not correct
    for (int i = 0; i < joint_names_.size(); i++){
      // besides its name, this method only changes a single joint position...
      goal_state_->setJointPositions(joint_names_[i], &initial_joint_position_[i]);
    }
  }

  void ThrowIK::set_bio_ik_timeout(double timeout){
    bio_ik_timeout_ = timeout;
  }
} //bitbots_throw