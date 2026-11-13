#include <libobsensor/ObSensor.hpp>

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string json_string(const std::string & value)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char character : value) {
    if (character == '"' || character == '\\') out << '\\' << character;
    else if (character < 0x20U) {
      const char * digits = "0123456789abcdef";
      out << "\\u00" << digits[character >> 4U] << digits[character & 15U];
    } else out << character;
  }
  out << '"';
  return out.str();
}

enum class PropertyKind { integer, boolean, roi, hdr };

void print_properties(const std::shared_ptr<ob::Device> & device, const std::string & serial)
{
  struct Property { const char * name; OBPropertyID id; PropertyKind kind; };
  const Property properties[] = {
    {"depth_auto_exposure", OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, PropertyKind::boolean},
    {"ir_auto_exposure", OB_PROP_IR_AUTO_EXPOSURE_BOOL, PropertyKind::boolean},
    {"depth_exposure", OB_PROP_DEPTH_EXPOSURE_INT, PropertyKind::integer},
    {"ir_exposure", OB_PROP_IR_EXPOSURE_INT, PropertyKind::integer},
    {"depth_gain", OB_PROP_DEPTH_GAIN_INT, PropertyKind::integer},
    {"ir_gain", OB_PROP_IR_GAIN_INT, PropertyKind::integer},
    {"ir_ae_max_exposure", OB_PROP_IR_AE_MAX_EXPOSURE_INT, PropertyKind::integer},
    {"ir_brightness", OB_PROP_IR_BRIGHTNESS_INT, PropertyKind::integer},
    {"depth_ae_roi", OB_STRUCT_DEPTH_AE_ROI, PropertyKind::roi},
    {"laser_power_level", OB_PROP_LASER_POWER_LEVEL_CONTROL_INT, PropertyKind::integer},
    {"laser_power_actual_level", OB_PROP_LASER_POWER_ACTUAL_LEVEL_INT, PropertyKind::integer},
    {"laser_control", OB_PROP_LASER_CONTROL_INT, PropertyKind::integer},
    {"depth_hdr", OB_STRUCT_DEPTH_HDR_CONFIG, PropertyKind::hdr}};
  const auto info = device->getDeviceInfo();
  if (serial != info->getSerialNumber()) throw std::runtime_error("property device identity mismatch");
  std::cout << "{\"schema\":\"sunhorizon_camera_property_probe_v1\",\"read_only\":true,"
            << "\"capture_started\":false,\"serial\":" << json_string(serial)
            << ",\"device_name\":" << json_string(info->getName())
            << ",\"firmware\":" << json_string(info->getFirmwareVersion())
            << ",\"sdk\":" << json_string(std::to_string(ob::Version::getMajor()) + "." +
    std::to_string(ob::Version::getMinor()) + "." + std::to_string(ob::Version::getPatch()))
            << ",\"exposure_units\":\"SDK property units; no unit conversion assumed\",\"properties\":[";
  bool first = true;
  for (const auto & property : properties) {
    if (!first) std::cout << ',';
    first = false;
    std::cout << "{\"name\":" << json_string(property.name) << ",\"id\":" << property.id;
    bool readable = false;
    bool writable = false;
    // Independent support queries: a failed write query must not hide readable evidence.
    try {
      readable = device->isPropertySupported(property.id, OB_PERMISSION_READ);
      std::cout << ",\"read_supported\":" << (readable ? "true" : "false");
    } catch (const std::exception & error) {
      std::cout << ",\"read_support_error\":" << json_string(error.what());
    }
    try {
      writable = device->isPropertySupported(property.id, OB_PERMISSION_WRITE);
      std::cout << ",\"write_supported\":" << (writable ? "true" : "false");
    } catch (const std::exception & error) {
      std::cout << ",\"write_support_error\":" << json_string(error.what());
    }
    if (readable) {
      try {
        std::ostringstream value;
        if (property.kind == PropertyKind::integer) {
          value << device->getIntProperty(property.id);
        } else if (property.kind == PropertyKind::boolean) {
          value << (device->getBoolProperty(property.id) ? "true" : "false");
        } else if (property.kind == PropertyKind::roi) {
          OBRegionOfInterest roi{};
          uint32_t size = sizeof(roi);
          device->getStructuredData(property.id, reinterpret_cast<uint8_t *>(&roi), &size);
          if (size != sizeof(roi)) throw std::runtime_error("unexpected ROI structure size");
          value << "{\"left\":" << roi.x0_left << ",\"top\":" << roi.y0_top
                << ",\"right\":" << roi.x1_right << ",\"bottom\":" << roi.y1_bottom << '}';
        } else {
          OBHdrConfig hdr{};
          uint32_t size = sizeof(hdr);
          device->getStructuredData(property.id, reinterpret_cast<uint8_t *>(&hdr), &size);
          if (size != sizeof(hdr)) throw std::runtime_error("unexpected HDR structure size");
          value << "{\"enabled\":" << (hdr.enable ? "true" : "false")
                << ",\"exposure_1\":" << hdr.exposure_1 << ",\"gain_1\":" << hdr.gain_1
                << ",\"exposure_2\":" << hdr.exposure_2 << ",\"gain_2\":" << hdr.gain_2 << '}';
        }
        std::cout << ",\"actual\":" << value.str();
      } catch (const std::exception & error) {
        std::cout << ",\"read_error\":" << json_string(error.what());
      }
      if (property.kind == PropertyKind::integer) {
        try {
          const auto range = device->getIntPropertyRange(property.id);
          std::cout << ",\"range\":{\"min\":" << range.min << ",\"max\":" << range.max
                    << ",\"step\":" << range.step << ",\"default\":" << range.def << '}';
        } catch (const std::exception & error) {
          std::cout << ",\"range_error\":" << json_string(error.what());
        }
      }
    }
    std::cout << '}';
  }
  std::cout << "]}\n";
}

struct VideoProfile
{
  std::uint32_t width;
  std::uint32_t height;
  std::uint32_t fps;
  int format;
};

VideoProfile describe(const std::shared_ptr<ob::StreamProfile> & profile)
{
  const auto video = profile->as<ob::VideoStreamProfile>();
  return {
    video->getWidth(), video->getHeight(), video->getFps(),
    static_cast<int>(video->getFormat())};
}

void print_profile(
  const std::string & record,
  const std::string & serial,
  const VideoProfile & profile)
{
  std::cout << record << '\t' << serial << '\t'
            << profile.width << '\t' << profile.height << '\t'
            << profile.fps << '\t' << profile.format << '\n';
}

std::shared_ptr<ob::Device> unique_device_by_serial(
  const std::shared_ptr<ob::DeviceList> & devices,
  const std::string & expected_serial)
{
  std::vector<std::uint32_t> matches;
  for (std::uint32_t index = 0; index < devices->getCount(); ++index) {
    if (expected_serial == devices->getSerialNumber(index)) {
      matches.push_back(index);
    }
  }
  if (matches.size() != 1U) {
    throw std::runtime_error(
            "expected exactly one device for serial " + expected_serial +
            ", observed " + std::to_string(matches.size()));
  }
  return devices->getDevice(matches.front());
}

void print_profiles(
  const std::shared_ptr<ob::Pipeline> & pipeline,
  const std::string & serial,
  OBSensorType sensor,
  const std::string & record)
{
  const auto profiles = pipeline->getStreamProfileList(sensor);
  for (std::uint32_t index = 0; index < profiles->getCount(); ++index) {
    print_profile(record, serial, describe(profiles->getProfile(index)));
  }
}

void print_hardware_d2c_profiles(
  const std::shared_ptr<ob::Pipeline> & pipeline,
  const std::string & serial)
{
  const auto colors = pipeline->getStreamProfileList(OB_SENSOR_COLOR);
  for (std::uint32_t color_index = 0; color_index < colors->getCount(); ++color_index) {
    const auto color = colors->getProfile(color_index);
    const auto color_record = describe(color);
    const auto depths = pipeline->getD2CDepthProfileList(color, ALIGN_D2C_HW_MODE);
    for (std::uint32_t depth_index = 0; depth_index < depths->getCount(); ++depth_index) {
      const auto depth_record = describe(depths->getProfile(depth_index));
      std::cout << "hardware_d2c" << '\t' << serial << '\t'
                << color_record.width << '\t' << color_record.height << '\t'
                << color_record.fps << '\t' << color_record.format << '\t'
                << depth_record.width << '\t' << depth_record.height << '\t'
                << depth_record.fps << '\t' << depth_record.format << '\n';
    }
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  const bool property_mode = argc == 4 && std::string(argv[3]) == "--properties";
  if ((argc != 3 && !property_mode) || std::string(argv[1]) != "--serial" || std::string(argv[2]).empty()) {
    std::cerr << "usage: camera_profile_probe --serial <usb-serial> [--properties]\n";
    return 2;
  }
  try {
    const std::string serial = argv[2];
    auto context = std::make_shared<ob::Context>();
    auto device = unique_device_by_serial(context->queryDeviceList(), serial);
    if (property_mode) {
      print_properties(device, serial);
      return 0;
    }
    auto pipeline = std::make_shared<ob::Pipeline>(device);
    std::cout << "schema\tsunhorizon_camera_profile_probe_v1\n";
    std::cout << "sdk\t" << ob::Version::getMajor() << '.' << ob::Version::getMinor() << '.'
              << ob::Version::getPatch() << '\n';
    std::cout << "global_timestamp_supported\t" << serial << '\t'
              << (device->isGlobalTimestampSupported() ? "true" : "false") << '\n';
    print_profiles(pipeline, serial, OB_SENSOR_COLOR, "color");
    print_profiles(pipeline, serial, OB_SENSOR_DEPTH, "depth");
    print_hardware_d2c_profiles(pipeline, serial);
    return 0;
  } catch (const ob::Error & error) {
    std::cerr << "camera profile SDK failure: " << error.what() << '\n';
  } catch (const std::exception & error) {
    std::cerr << "camera profile failure: " << error.what() << '\n';
  }
  return 1;
}
