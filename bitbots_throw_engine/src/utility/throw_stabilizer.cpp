#include "utility/throw_stabilizer.h"

namespace bitbots_throw{
  ThrowStabilizer::ThrowStabilizer(){
    reset();
  }

  void ThrowStabilizer::reset(){
  }

  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ThrowStabilizer::stabilize(const ThrowResponse & response){
    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_options(new bio_ik::BioIKKinematicsQueryOptions());
    ik_options->replace = true;
    ik_options->return_approximate_solution = true;

    // trunk goal
    auto * trunk_goal = create_pose_goal(response.support_foot_to_trunk_
                                        ,"base_link"
                                        ,"l_sole"
                                        ,1); //TODO: rework maybe
    ik_options->goals.emplace_back(trunk_goal);

    // left hand goal
    auto * left_hand_goal = create_pose_goal(response.support_foot_to_left_hand_
                                            ,"base_link"
                                            ,"l_sole"
                                            ,1); //TODO: rework maybe
    ik_options->goals.emplace_back(left_hand_goal);

    // right hand goal
    auto * right_hand_goal = create_pose_goal(response.support_foot_to_right_hand_
                                             ,"base_link"
                                             ,"r_sole"
                                             ,1); //TODO: rework maybe
    ik_options->goals.emplace_back(right_hand_goal);

    // left feet goal
    auto * left_feet_goal = create_pose_goal(response.support_foot_to_left_foot_
                                            ,"base_link"
                                            ,"r_sole"
                                            ,1); //TODO: rework maybe
    ik_options->goals.emplace_back(left_feet_goal);

    // right hand goal
    auto * right_feet_goal = create_pose_goal(response.support_foot_to_right_foot_
                                             ,"base_link"
                                             ,"l_sole"
                                             ,1); //TODO: rework maybe
    ik_options->goals.emplace_back(right_feet_goal);

    return std::move(ik_options);
  }

  ReferencePoseGoal * ThrowStabilizer::create_pose_goal(tf2::Transform const & support_foot_to
                                                       ,std::string const & link_name
                                                       ,std::string const & reference_link_name
                                                       ,double const & weight){
      auto * goal = new ReferencePoseGoal();
      goal->setPosition(support_foot_to.getOrigin());
      goal->setOrientation(support_foot_to.getRotation());
      goal->setLinkName(link_name);
      goal->setReferenceLinkName(reference_link_name); //TODO: rework maybe
      goal->setWeight(weight);
      return goal;
  }
} //bitbots_throw