#ifndef THROW_STABILIZER_H
#define THROW_STABILIZER_H

#include <bio_ik/bio_ik.h>
#include "throws/throw_curves/throw_curve.h"

class ThrowStabilizer
{
public:
  WalkStabilizer();
  virtual void reset();
  virtual std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> stabilize(const std::shared_ptr<ThrowCurve> throw_curve);

private:
};

#endif