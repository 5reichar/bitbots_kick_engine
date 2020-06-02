#include "utility/throw_stabilizer.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_throw{
  ThrowStabilizer::ThrowStabilizer(){
    reset();
  }

  void ThrowStabilizer::reset(){
  }

  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ThrowStabilizer::stabilize(const ThrowResponse & response){
    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_options;
    ik_options->replace = true;
    ik_options->return_approximate_solution = true;

    // trunk goal
    auto *trunk_goal = new ReferencePoseGoal();
    trunk_goal->setPosition(response.support_foot_to_trunk_.getOrigin());
    trunk_goal->setOrientation(response.support_foot_to_trunk_.getRotation());
    trunk_goal->setLinkName("base_link");
    trunk_goal->setReferenceLinkName("l_sole"); //TODO: rework maybe
    trunk_goal->setWeight(1);
    ik_options->goals.emplace_back(trunk_goal);

    // left hand goal
    auto *left_hand_goal = new ReferencePoseGoal();
    left_hand_goal->setPosition(response.support_foot_to_left_hand_.getOrigin());
    left_hand_goal->setOrientation(response.support_foot_to_left_hand_.getRotation());
    left_hand_goal->setLinkName("base_link");
    left_hand_goal->setReferenceLinkName("l_sole"); //TODO: rework maybe
    left_hand_goal->setWeight(1);
    ik_options->goals.emplace_back(left_hand_goal);

    // right hand goal
    auto *right_hand_goal = new ReferencePoseGoal();
    right_hand_goal->setPosition(response.support_foot_to_right_hand_.getOrigin());
    right_hand_goal->setOrientation(response.support_foot_to_right_hand_.getRotation());
    right_hand_goal->setLinkName("base_link");
    right_hand_goal->setReferenceLinkName("r_sole"); //TODO: rework maybe
    right_hand_goal->setWeight(1);
    ik_options->goals.emplace_back(right_hand_goal);

    return std::move(ik_options);
  }
} //bitbots_throw