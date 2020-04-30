#ifndef THROW_STABILIZER_H
#define THROW_STABILIZER_H

#include "throws/throw_curves/throw_curve.h"
#include "../../bitbots_spline/include/utils/abstract_stabilizer.h"

class ThrowStabilizer : public bitbots_splines::AbstractStabilizer<std::shared_ptr<ThrowCurve>>
{
public:
  WalkStabilizer();
  virtual void reset() override;
  virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const std::shared_ptr<ThrowCurve> &positions) override;

private:
};

#endif