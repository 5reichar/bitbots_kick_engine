#ifndef THROW_STABILIZER_H
#define THROW_STABILIZER_H

#include <memory>
#include "utility/throw_utilities.h"
#include "bitbots_splines/abstract_stabilizer.h"

class ThrowStabilizer : public bitbots_splines::AbstractStabilizer<ThrowResponse>
{
public:
  ThrowStabilizer();
  virtual void reset() override;
  virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const ThrowResponse & response) override;

private:
};

#endif