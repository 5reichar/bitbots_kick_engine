#include "utility/throw_stabilizer.h"

namespace bitbots_throw{
  void ThrowStabilizer::reset(){
  }

  std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ThrowStabilizer::stabilize(const std::vector<ThrowStabilizerData> & data){
    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ik_options(new bio_ik::BioIKKinematicsQueryOptions());
    ik_options->replace = true;
    ik_options->return_approximate_solution = true;

    for(auto const & it : data){
        auto * goal = new ReferencePoseGoal();
        goal->setPosition(it.transform_to_support_.getOrigin());
        goal->setOrientation(it.transform_to_support_.getRotation());
        goal->setLinkName(it.link_name_);
        goal->setReferenceLinkName(it.reference_link_name_); //TODO: rework maybe
        goal->setWeight(it.weight_);
        ik_options->goals.emplace_back(goal);
    }

    return std::move(ik_options);
  }
} //bitbots_throw