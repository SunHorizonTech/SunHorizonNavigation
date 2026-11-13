#pragma once

namespace sunhorizon_rgbd_adapter {

double measurement_age_seconds(double now_sec, double measurement_stamp_sec);
bool measurement_fresh(double age_sec, double freshness_timeout_sec);

}  // namespace sunhorizon_rgbd_adapter
