#ifndef BITBOTS_THROW_THROW_STABILIZER_H
#define BITBOTS_THROW_THROW_STABILIZER_H

#include <memory>
#include "utility/throw_utilities.h"
#include "bitbots_splines/abstract_stabilizer.h"
#include "bitbots_splines/reference_goals.h"

namespace bitbots_throw{
  struct ThrowStabilizerData{
      tf2::Transform transform_to_support_;
      std::string link_name_;
      std::string reference_link_name_;
      double weight_;
  };

  class ThrowStabilizer : public bitbots_splines::AbstractStabilizer<std::vector<ThrowStabilizerData>>{
  public:
    void reset() override;
    std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const std::vector<ThrowStabilizerData> & data) override;
  };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_STABILIZER_H