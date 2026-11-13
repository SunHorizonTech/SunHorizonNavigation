// Bounded, file-only stereo diagnostic. Requires exclusive camera ownership.
// No ROS node, actuator interface, calibration writes, or persistent settings.
#include <libobsensor/ObSensor.hpp>
#include <array>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

struct Channel {
  const char * name;
  OBSensorType sensor;
  OBFrameType frame;
  OBFormat format;
  unsigned bytes;
  std::shared_ptr<ob::VideoStreamProfile> profile;
};

int main(int argc, char ** argv) {
  if (argc != 5 || std::string(argv[1]) != "--serial" ||
      std::string(argv[3]) != "--output") {
    std::cerr << "usage: camera_static_capture --serial SERIAL --output NEW_DIRECTORY\n";
    return 2;
  }
  try {
    const std::string serial = argv[2];
    if (serial.empty() || serial.size() > 64) throw std::runtime_error("invalid serial");
    for (unsigned char c : serial) {
      if (!std::isalnum(c)) throw std::runtime_error("invalid serial character");
    }
    const fs::path output = argv[4];
    if (!output.is_absolute() || !fs::create_directory(output)) {
      throw std::runtime_error("output must be a new absolute directory with existing parent");
    }
    ob::Context context;
    auto devices = context.queryDeviceList();
    std::shared_ptr<ob::Device> device;
    unsigned matches = 0;
    for (uint32_t i = 0; i < devices->getCount(); ++i) {
      if (serial == devices->getSerialNumber(i)) {
        ++matches;
        device = devices->getDevice(i);
      }
    }
    if (matches != 1 || serial != device->getDeviceInfo()->getSerialNumber()) {
      throw std::runtime_error("camera identity mismatch");
    }
    ob::Pipeline pipeline(device);
    std::array<Channel, 4> channels{{
      {"depth", OB_SENSOR_DEPTH, OB_FRAME_DEPTH, OB_FORMAT_Y16, 2, {}},
      {"left", OB_SENSOR_IR_LEFT, OB_FRAME_IR_LEFT, OB_FORMAT_Y8, 1, {}},
      {"right", OB_SENSOR_IR_RIGHT, OB_FRAME_IR_RIGHT, OB_FORMAT_Y8, 1, {}},
      {"color", OB_SENSOR_COLOR, OB_FRAME_COLOR, OB_FORMAT_RGB, 3, {}}
    }};
    auto config = std::make_shared<ob::Config>();
    std::ofstream calibration(output / "calibration.ndjson");
    calibration.exceptions(std::ios::badbit | std::ios::failbit);
    calibration << std::setprecision(12);
    for (auto & ch : channels) {
      ch.profile = pipeline.getStreamProfileList(ch.sensor)->getVideoStreamProfile(
        640, 480, ch.format, 30);
      config->enableStream(ch.profile);
      const auto k = ch.profile->getIntrinsic();
      const auto d = ch.profile->getDistortion();
      calibration << "{\"stream\":\"" << ch.name << "\",\"format\":" << ch.format
        << ",\"width\":" << k.width << ",\"height\":" << k.height
        << ",\"fx\":" << k.fx << ",\"fy\":" << k.fy
        << ",\"cx\":" << k.cx << ",\"cy\":" << k.cy
        << ",\"distortion_model\":" << d.model << ",\"distortion_k1_k6_p1_p2\":["
        << d.k1 << ',' << d.k2 << ',' << d.k3 << ',' << d.k4 << ',' << d.k5 << ','
        << d.k6 << ',' << d.p1 << ',' << d.p2 << "]}\n";
    }
    for (const auto & ch : channels) {
      const auto e = channels[0].profile->getExtrinsicTo(ch.profile);
      calibration << "{\"from\":\"depth\",\"to\":\"" << ch.name << "\",\"rotation_sdk\":[";
      for (int i = 0; i < 9; ++i) calibration << (i ? "," : "") << e.rot[i];
      calibration << "],\"translation_mm\":[";
      for (int i = 0; i < 3; ++i) calibration << (i ? "," : "") << e.trans[i];
      calibration << "]}\n";
    }
    calibration.close();
    config->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    pipeline.enableFrameSync();
    pipeline.start(config);
    std::ofstream frames(output / "frames.ndjson");
    frames.exceptions(std::ios::badbit | std::ios::failbit);
    const auto start = Clock::now();
    unsigned received = 0, saved = 0;
    uint64_t previous_depth_us = 0;
    while (Clock::now() - start < std::chrono::seconds(14) && saved < 30) {
      auto set = pipeline.waitForFrameset(500);
      if (!set) continue;
      ++received;
      if (Clock::now() - start < std::chrono::seconds(4) || received % 3) continue;
      const auto depth_frame = set->getDepthFrame();
      if (!depth_frame) throw std::runtime_error("frameset missing depth");
      const auto depth_us = depth_frame->getTimeStampUs();
      if (depth_us <= previous_depth_us) throw std::runtime_error("depth time did not advance");
      previous_depth_us = depth_us;
      for (const auto & ch : channels) {
        auto frame = set->getFrame(ch.frame);
        if (!frame) throw std::runtime_error("incomplete frameset");
        auto video = frame->as<ob::VideoFrame>();
        if (video->getWidth() != 640 || video->getHeight() != 480 ||
            frame->getFormat() != ch.format || frame->getDataSize() != 640 * 480 * ch.bytes) {
          throw std::runtime_error("unexpected frame layout");
        }
        const auto filename = std::to_string(saved) + "-" + ch.name + ".bin";
        std::ofstream raw(output / filename, std::ios::binary);
        raw.exceptions(std::ios::badbit | std::ios::failbit);
        raw.write(reinterpret_cast<const char *>(frame->getData()), frame->getDataSize());
        raw.close();
        frames << "{\"index\":" << saved << ",\"stream\":\"" << ch.name
          << "\",\"file\":\"" << filename << "\",\"device_us\":" << frame->getTimeStampUs()
          << ",\"system_us\":" << frame->getSystemTimeStampUs()
          << ",\"global_us\":" << frame->getGlobalTimeStampUs()
          << ",\"bytes\":" << frame->getDataSize();
        if (ch.frame == OB_FRAME_DEPTH) {
          frames << ",\"depth_scale_mm\":" << frame->as<ob::DepthFrame>()->getValueScale();
        }
        frames << "}\n";
      }
      frames.flush();
      ++saved;
    }
    pipeline.stop();
    if (saved != 30) throw std::runtime_error("capture incomplete");
    std::ofstream complete(output / "complete.json");
    complete.exceptions(std::ios::badbit | std::ios::failbit);
    complete << "{\"complete\":true,\"serial\":\"" << serial
      << "\",\"saved_framesets\":" << saved << ",\"received_framesets\":" << received
      << ",\"alignment\":\"disabled\",\"purpose\":\"stationary raw stereo diagnostic only\"}\n";
    complete.close();
    std::cout << "Captured " << saved << " stereo/depth/color sets\n";
    return 0;
  } catch (const ob::Error & e) {
    std::cerr << "SDK capture failed: " << e.what() << '\n';
  } catch (const std::exception & e) {
    std::cerr << "Capture failed: " << e.what() << '\n';
  }
  return 1;
}
