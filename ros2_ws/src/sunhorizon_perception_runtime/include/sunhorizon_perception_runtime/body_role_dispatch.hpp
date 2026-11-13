#pragma once
#include <cstddef>
#include <stdexcept>

namespace sunhorizon_perception_runtime::detail {
// Preserve belly's spatial anchor. A chest without a spatial contribution owns
// an independent latest-frame lane; it must not be queued behind a belly epoch.
// True dual-source spatial operation retains the original bounded chest history.
template<class Work,class Belly,class BodyOnlyChest,class RetainChest>
void dispatch_prepared_body(std::size_t role,bool include_chest,const Work& work,
    Belly belly,BodyOnlyChest body_only_chest,RetainChest retain_chest) {
  if(role==0)belly(work);
  else if(role!=1)throw std::invalid_argument("body_dispatch_invalid_role");
  else if(include_chest)retain_chest(work);
  else body_only_chest(work);
}
} // namespace sunhorizon_perception_runtime::detail
