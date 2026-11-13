#include "sunhorizon_rgbd_adapter/body_geometry_cuda.hpp"
#include <stdexcept>
namespace sunhorizon_rgbd_adapter {
struct BodyGeometryCuda::Impl {};
bool BodyGeometryCuda::compiled() noexcept {return false;}
BodyGeometryCuda::BodyGeometryCuda(bool){throw std::runtime_error("camera_body_cuda_not_compiled");}
BodyGeometryCuda::~BodyGeometryCuda()=default;
bool BodyGeometryCuda::profiling()const noexcept{return false;}
bool BodyGeometryCuda::has_pending_device_work()const{return false;}
BodyGeometryCuda::Timings BodyGeometryCuda::last_timings()const{return {}; }
void BodyGeometryCuda::request_stop() noexcept {}
void BodyGeometryCuda::query(const Model&,double,const Pixel*,const std::vector<Sample>&,
    const std::function<bool(std::size_t,const Result&)>&){throw std::runtime_error("camera_body_cuda_not_compiled");}
}
