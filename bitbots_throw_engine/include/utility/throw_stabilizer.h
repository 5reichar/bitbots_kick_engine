#ifndef BITBOTS_THROW_THROW_STABILIZER_H
#define BITBOTS_THROW_THROW_STABILIZER_H

#include <memory>
#include "utility/throw_utilities.h"
#include "bitbots_splines/abstract_stabilizer.h"

namespace bitbots_throw{
  class ThrowStabilizer : public bitbots_splines::AbstractStabilizer<ThrowResponse>{
  public:
    ThrowStabilizer();
    virtual void reset() override;
    virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const ThrowResponse & response) override;

  private:
  };
} //bitbots_throw
#endif //BITBOTS_THROW_THROW_STABILIZER_H