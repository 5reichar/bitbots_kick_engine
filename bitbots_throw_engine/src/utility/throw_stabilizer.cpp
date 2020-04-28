#include "utility/throw_stabilizer.h"

ThrowStabilizer::ThrowStabilizer() {
  reset();
}

void ThrowStabilizer::reset() {
}

std::unique_ptr<bio_ik::BioIKKinematicsQueryOptions> ThrowStabilizer::stabilize(const std::shared_ptr<ThrowCurve> throw_curve) {
  auto ik_options = std::make_unique<bio_ik::BioIKKinematicsQueryOptions>();
  ik_options->replace = true;
  ik_options->return_approximate_solution = true;

  return std::move(ik_options);
}