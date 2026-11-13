#pragma once
namespace sunhorizon_obstacle_coverage::dynamic_self_geometry {
// True only for the CPU geometry library rebuilt with the exact reviewed
// CUDA counterpart's arithmetic contract. Default builds return false.
bool camera_cuda_arithmetic_contract_supported() noexcept;
}
