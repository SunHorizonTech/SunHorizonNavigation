#pragma once
#include <cstddef>

namespace sunhorizon_3d_localizer {
// Drain queued priors before expensive registration. Validate every sample;
// never skip a discontinuity to reach a newer apparently healthy pose.
// At the bound, defer matching until another callback can finish draining.
template <typename Sample, typename Take, typename Ingest>
bool drain_prior_samples(Take take, Ingest ingest, std::size_t limit) {
  for (std::size_t count = 0; count < limit; ++count) {
    Sample sample;
    if (!take(sample)) return true;
    if (!ingest(sample)) return false;
  }
  return false;
}
}  // namespace sunhorizon_3d_localizer
