#ifndef BITBOTS_THROW_THROW_STABILIZER_H
#define BITBOTS_THROW_THROW_STABILIZER_H

#include <memory>
#include "utility/throw_utilities.h"
#include "bitbots_splines/abstract_stabilizer.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_throw{
  class ThrowStabilizer : public bitbots_splines::AbstractStabilizer<ThrowResponse>{
  public:
    ThrowStabilizer();
    void reset() override;
    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const ThrowResponse & response) override;

  private:
      ReferencePoseGoal * create_pose_goal(tf2::Transform const & support_foot_to,
                                           std::string const & link_name,
                                           std::string const & reference_link_name,
                                           double const & weight);
  };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_STABILIZER_H