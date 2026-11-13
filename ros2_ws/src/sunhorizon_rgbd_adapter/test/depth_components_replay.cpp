// Standalone real-frame validator: RAW16 WIDTH HEIGHT STEP BIGENDIAN DELTA MIN MASK_OUT
#include "sunhorizon_rgbd_adapter/depth_components.hpp"
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 9) return 2;
  std::ifstream input(argv[1], std::ios::binary);
  if (!input) throw std::runtime_error("depth_input_unavailable");
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
  auto begin=std::chrono::steady_clock::now();
  auto result=sunhorizon_rgbd_adapter::depth_component_mask(
      {bytes.data(),bytes.size(),static_cast<uint32_t>(std::stoul(argv[2])),
       static_cast<uint32_t>(std::stoul(argv[3])),static_cast<uint32_t>(std::stoul(argv[4])),
       std::stoi(argv[5])!=0},.001,.2,8.,std::stod(argv[6]),std::stoul(argv[7]));
  const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
  if (!result.valid) throw std::runtime_error(result.reason);
  std::ofstream output(argv[8],std::ios::binary);
  output.write(reinterpret_cast<const char*>(result.keep.data()),result.keep.size());
  if (!output) throw std::runtime_error("mask_output_failed");
  std::cout << "{\"valid_pixels\":" << result.valid_pixels
      << ",\"removed_pixels\":" << result.removed_pixels
      << ",\"removed_components\":" << result.removed_components
      << ",\"wall_ms\":" << ms << "}\n";
}
