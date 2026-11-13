#include "sunhorizon_obstacle_coverage/coverage_evaluator.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>
#include <utility>

using namespace sunhorizon_obstacle_coverage;

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
void contains(const std::string& value, const char* expected) {
  require(value.find(expected) != std::string::npos, expected);
}
bool equal(double a, double b) { return a == b || (std::isnan(a) && std::isnan(b)); }
void same(const SourceObservation& a, const SourceObservation& b) {
  require(a.seen == b.seen && a.healthy == b.healthy && equal(a.stamp_sec,b.stamp_sec) &&
          equal(a.receipt_stamp_sec,b.receipt_stamp_sec), "diagnostics changed observation");
}
int main() {
  const SourceObservation good{true,true,9.95,9.98};
  // Both sources can independently explain the exact same merged reason.
  for (bool stale_raw : {true,false}) {
    const SourceObservation delayed{true,true,9.697,9.965};
    const auto raw = stale_raw ? delayed : good;
    const auto processed = stale_raw ? good : delayed;
    const auto text = processed_observation_diagnostics(raw,processed,10.,.3,.3,true);
    contains(text,stale_raw ? "branch=upstream_rejected" : "branch=processed_rejected");
    contains(text,"header_age=0.303000000,receipt_age=0.035000000");
    contains(text,"state=queue_delay");
    CoverageInputs input;input.now_sec=10.;input.chest=good;
    input.belly=require_processed_observation(raw,processed,10.,.3,.3);
    require(CoverageEvaluator(CoverageProfile::kVisual).evaluate(input).reason=="belly_queue_delay",
            "original rejection changed");
  }
  // Upstream wins if both reject; receipt/source minima can come from different inputs.
  contains(processed_observation_diagnostics({}, {}, 10.,.3,.3,true),"branch=upstream_rejected");
  auto combined=processed_observation_diagnostics({true,true,9.8,9.99},{true,true,9.9,9.85},10.,.3,.3,true);
  contains(combined,"branch=combined_min,stamp_from=raw,receipt_from=processed");
  contains(processed_observation_diagnostics(good,good,10.,.3,.3,true),"stamp_from=equal,receipt_from=equal");
  contains(processed_observation_diagnostics(good,{},10.,.3,.3,false),"branch=upstream_only");
  // A looser raw threshold cannot hide a rejected processed timeline.
  const auto strict=processed_observation_diagnostics(good,{true,true,9.5,9.99},10.,1.,.3,true);
  contains(strict,"branch=processed_rejected");contains(strict,"freshness=1.000000000");
  contains(strict,"freshness=0.300000000");
  const std::vector<std::pair<SourceObservation,const char*>> cases{
    {{},"state=missing"},{{true,true,0.,9.9},"state=invalid_header_stamp"},
    {{true,true,10.1,9.9},"state=future_header_stamp"},
    {{true,true,9.9,0.},"state=invalid_receipt_stamp"},
    {{true,true,9.9,10.1},"state=future_receipt_stamp"},
    {{true,true,9.5,9.5},"state=stale"},{{true,true,9.9,9.5},"state=receipt_stale"},
    {{true,false,9.9,9.9},"state=unhealthy"},
    {{true,true,std::numeric_limits<double>::quiet_NaN(),9.9},"state=invalid_header_stamp"}};
  for (const auto& entry:cases) {
    const auto raw=entry.first;const auto before=require_processed_observation(raw,good,10.,.3,.3);
    const auto text=processed_observation_diagnostics(raw,good,10.,.3,.3,true);
    contains(text,entry.second);same(raw,entry.first);
    same(before,require_processed_observation(raw,good,10.,.3,.3));
  }
  std::cout << "processed observation diagnostics: all deterministic cases passed\n";
}
