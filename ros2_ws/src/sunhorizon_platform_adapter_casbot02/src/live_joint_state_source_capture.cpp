#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fcntl.h>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/joint_state.hpp>

namespace {

constexpr char kSourceTopic[] = "/joint_states";
constexpr char kSourceType[] = "sensor_msgs/msg/JointState";
constexpr char kSourceOwnerName[] = "hl_orin_core_msg";
constexpr char kSourceOwnerNamespace[] = "/";
constexpr char kSchema[] = "sunhorizon.robot_a.live_joint_state_capture.v1";
constexpr double kMinimumDurationSec = 10.0;
constexpr double kMaximumDurationSec = 300.0;
constexpr double kMaximumGraphTimeoutSec = 60.0;
constexpr double kCaptureOverrunSec = 10.0;
constexpr std::size_t kMaximumSampleCount = 100000U;

struct Arguments {
  std::filesystem::path output_path;
  double duration_sec{0.0};
  double graph_timeout_sec{0.0};
};

struct PublisherIdentity {
  std::string node_name;
  std::string node_namespace;
  std::string topic_type;
  std::string endpoint_gid;

  bool operator==(const PublisherIdentity& other) const {
    return node_name == other.node_name &&
           node_namespace == other.node_namespace &&
           topic_type == other.topic_type && endpoint_gid == other.endpoint_gid;
  }
};

struct Sample {
  std::vector<std::string> names;
  std::vector<double> positions;
  int64_t stamp_ns{0};
  int64_t received_ros_time_ns{0};
  int64_t received_monotonic_ns{0};
  std::string publisher_endpoint_gid;
};

double strict_double(const std::string& value, const char* reason) {
  std::size_t consumed = 0U;
  double parsed = 0.0;
  try {
    parsed = std::stod(value, &consumed);
  } catch (const std::exception&) {
    throw std::runtime_error(reason);
  }
  if (consumed != value.size() || !std::isfinite(parsed)) {
    throw std::runtime_error(reason);
  }
  return parsed;
}

Arguments parse_arguments(int argc, char** argv) {
  if (argc != 7) throw std::runtime_error("capture_arguments_invalid");
  std::optional<std::string> output;
  std::optional<std::string> duration;
  std::optional<std::string> graph_timeout;
  for (int index = 1; index < argc; index += 2) {
    const std::string option(argv[index]);
    const std::string value(argv[index + 1]);
    if (option == "--output" && !output) {
      output = value;
    } else if (option == "--duration-sec" && !duration) {
      duration = value;
    } else if (option == "--graph-timeout-sec" && !graph_timeout) {
      graph_timeout = value;
    } else {
      throw std::runtime_error("capture_arguments_invalid");
    }
  }
  if (!output || !duration || !graph_timeout) {
    throw std::runtime_error("capture_arguments_invalid");
  }
  Arguments arguments;
  arguments.output_path = *output;
  arguments.duration_sec = strict_double(*duration, "capture_duration_invalid");
  arguments.graph_timeout_sec =
      strict_double(*graph_timeout, "capture_graph_timeout_invalid");
  if (!arguments.output_path.is_absolute() ||
      arguments.duration_sec < kMinimumDurationSec ||
      arguments.duration_sec > kMaximumDurationSec ||
      arguments.graph_timeout_sec <= 0.0 ||
      arguments.graph_timeout_sec > kMaximumGraphTimeoutSec) {
    throw std::runtime_error("capture_arguments_invalid");
  }
  return arguments;
}

std::string gid_bytes_hex(const uint8_t* bytes, std::size_t size) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0U; index < size; ++index) {
    output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return output.str();
}

std::string endpoint_gid_hex(const rmw_gid_t& gid) {
  return gid_bytes_hex(gid.data, RMW_GID_STORAGE_SIZE);
}

template <std::size_t Size>
std::string endpoint_gid_hex(const std::array<uint8_t, Size>& gid) {
  return gid_bytes_hex(gid.data(), gid.size());
}

PublisherIdentity current_identity(const rclcpp::Node& node) {
  const auto endpoints = node.get_publishers_info_by_topic(kSourceTopic);
  if (endpoints.size() != 1U) {
    throw std::runtime_error(
        "source_publisher_count_invalid:" + std::to_string(endpoints.size()));
  }
  const auto& endpoint = endpoints.front();
  PublisherIdentity identity{
      endpoint.node_name(), endpoint.node_namespace(), endpoint.topic_type(),
      endpoint_gid_hex(endpoint.endpoint_gid())};
  if (identity.node_name == "_NODE_NAME_UNKNOWN_" ||
      identity.node_namespace == "_NODE_NAMESPACE_UNKNOWN_") {
    throw std::runtime_error("source_publisher_identity_discovery_pending");
  }
  if (identity.node_name != kSourceOwnerName ||
      identity.node_namespace != kSourceOwnerNamespace ||
      identity.topic_type != kSourceType ||
      identity.endpoint_gid.size() != RMW_GID_STORAGE_SIZE * 2U) {
    throw std::runtime_error("source_publisher_identity_invalid");
  }
  return identity;
}

PublisherIdentity wait_for_identity(
    const std::shared_ptr<rclcpp::Node>& node, double graph_timeout_sec) {
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(graph_timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      return current_identity(*node);
    } catch (const std::runtime_error& error) {
      const std::string reason(error.what());
      if (reason.rfind("source_publisher_count_invalid:", 0U) != 0U &&
          reason != "source_publisher_identity_discovery_pending") {
        throw;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  throw std::runtime_error("source_publisher_discovery_timeout");
}

void append_json_string(std::ostringstream& output, const std::string& value) {
  output << '"';
  for (const unsigned char character : value) {
    switch (character) {
      case '"': output << "\\\""; break;
      case '\\': output << "\\\\"; break;
      case '\b': output << "\\b"; break;
      case '\f': output << "\\f"; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default:
        if (character < 0x20U) {
          output << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                 << static_cast<unsigned int>(character) << std::dec;
        } else {
          output << static_cast<char>(character);
        }
    }
  }
  output << '"';
}

std::string capture_json(
    const PublisherIdentity& identity, const std::vector<Sample>& samples) {
  std::ostringstream output;
  output << std::setprecision(17);
  output << "{\"schema\":";
  append_json_string(output, kSchema);
  output << ",\"source\":{\"node_name\":";
  append_json_string(output, identity.node_name);
  output << ",\"node_namespace\":";
  append_json_string(output, identity.node_namespace);
  output << ",\"topic_type\":";
  append_json_string(output, identity.topic_type);
  output << ",\"endpoint_gid\":";
  append_json_string(output, identity.endpoint_gid);
  output << "},\"samples\":[";
  for (std::size_t sample_index = 0U; sample_index < samples.size(); ++sample_index) {
    if (sample_index != 0U) output << ',';
    const auto& sample = samples[sample_index];
    output << "{\"names\":[";
    for (std::size_t index = 0U; index < sample.names.size(); ++index) {
      if (index != 0U) output << ',';
      append_json_string(output, sample.names[index]);
    }
    output << "],\"positions\":[";
    for (std::size_t index = 0U; index < sample.positions.size(); ++index) {
      if (index != 0U) output << ',';
      output << sample.positions[index];
    }
    output << "],\"stamp_ns\":" << sample.stamp_ns
           << ",\"received_ros_time_ns\":" << sample.received_ros_time_ns
           << ",\"received_monotonic_ns\":" << sample.received_monotonic_ns
           << ",\"publisher_endpoint_gid\":";
    append_json_string(output, sample.publisher_endpoint_gid);
    output << '}';
  }
  output << "]}\n";
  return output.str();
}

void write_exclusive(const std::filesystem::path& path, const std::string& data) {
  const int descriptor = ::open(
      path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, S_IRUSR | S_IWUSR);
  if (descriptor < 0) throw std::runtime_error("capture_output_create_failed");
  bool complete = false;
  try {
    std::size_t offset = 0U;
    while (offset < data.size()) {
      const ssize_t written = ::write(
          descriptor, data.data() + offset, data.size() - offset);
      if (written <= 0) throw std::runtime_error("capture_output_write_failed");
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(descriptor) != 0) {
      throw std::runtime_error("capture_output_fsync_failed");
    }
    complete = true;
  } catch (...) {
    ::close(descriptor);
    ::unlink(path.c_str());
    throw;
  }
  if (::close(descriptor) != 0) {
    ::unlink(path.c_str());
    throw std::runtime_error("capture_output_close_failed");
  }
  if (!complete) throw std::runtime_error("capture_output_write_failed");
}

std::vector<Sample> capture(
    const std::shared_ptr<rclcpp::Node>& node,
    const PublisherIdentity& expected_identity,
    double duration_sec) {
  std::vector<Sample> samples;
  samples.reserve(static_cast<std::size_t>(duration_sec * 250.0));
  std::optional<std::string> failure;
  rclcpp::SensorDataQoS qos;
  qos.keep_last(1000);
  auto subscription = node->create_subscription<sensor_msgs::msg::JointState>(
      kSourceTopic, qos,
      [&](const sensor_msgs::msg::JointState::ConstSharedPtr message,
          const rclcpp::MessageInfo& message_info) {
        if (failure) return;
        try {
          if (!(current_identity(*node) == expected_identity)) {
            throw std::runtime_error("source_publisher_identity_changed");
          }
          const auto message_gid = endpoint_gid_hex(
              message_info.get_rmw_message_info().publisher_gid);
          if (message_gid != expected_identity.endpoint_gid) {
            throw std::runtime_error("source_message_endpoint_gid_mismatch");
          }
          if (samples.size() >= kMaximumSampleCount) {
            throw std::runtime_error("capture_sample_limit_exceeded");
          }
          const int64_t stamp_ns =
              rclcpp::Time(message->header.stamp).nanoseconds();
          const int64_t ros_now_ns = node->get_clock()->now().nanoseconds();
          const int64_t monotonic_now_ns =
              std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::steady_clock::now().time_since_epoch()).count();
          samples.push_back({
              message->name, message->position, stamp_ns, ros_now_ns,
              monotonic_now_ns, message_gid});
        } catch (const std::exception& error) {
          failure = error.what();
        }
      });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto wall_deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(duration_sec + kCaptureOverrunSec);
  auto next_graph_check = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    executor.spin_some();
    if (failure) throw std::runtime_error(*failure);
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_graph_check) {
      if (!(current_identity(*node) == expected_identity)) {
        throw std::runtime_error("source_publisher_identity_changed");
      }
      next_graph_check = now + std::chrono::milliseconds(50);
    }
    if (samples.size() >= 2U) {
      const double source_span_sec =
          static_cast<double>(samples.back().stamp_ns - samples.front().stamp_ns) / 1e9;
      const double receive_span_sec = static_cast<double>(
          samples.back().received_monotonic_ns -
          samples.front().received_monotonic_ns) / 1e9;
      if (source_span_sec >= duration_sec && receive_span_sec >= duration_sec) break;
    }
    if (now >= wall_deadline) {
      throw std::runtime_error("joint_state_capture_duration_timeout");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  executor.remove_node(node);
  if (!rclcpp::ok()) throw std::runtime_error("rclcpp_shutdown_during_capture");
  if (!(current_identity(*node) == expected_identity)) {
    throw std::runtime_error("source_publisher_identity_changed");
  }
  return samples;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto arguments = parse_arguments(argc, argv);
    if (std::filesystem::exists(arguments.output_path) ||
        !std::filesystem::is_directory(arguments.output_path.parent_path())) {
      throw std::runtime_error("capture_output_path_invalid");
    }
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "sunhorizon_live_joint_state_evidence_capture");
    const auto identity = wait_for_identity(node, arguments.graph_timeout_sec);
    const auto samples = capture(node, identity, arguments.duration_sec);
    write_exclusive(arguments.output_path, capture_json(identity, samples));
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    if (rclcpp::ok()) rclcpp::shutdown();
    std::cerr << error.what() << '\n';
    return 1;
  }
}
