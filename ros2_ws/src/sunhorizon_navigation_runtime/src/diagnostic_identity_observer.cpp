// Private, read-only MessageInfo bridge for Humble rclpy, which drops the GID.
#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <fcntl.h>
#include <iostream>
#include <memory>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <time.h>
#include <unistd.h>
#include <utility>

namespace sunhorizon_navigation_runtime::diagnostic_observer {
constexpr std::size_t kMaximumSampleBytes = 120 * 1024;
constexpr std::size_t kMaximumLineBytes = 128 * 1024;
constexpr char kDirectRow[] = "sunhorizon_navigation_runtime/direct_owner_snapshot_v1";
using Message = diagnostic_msgs::msg::DiagnosticArray;

enum class Mode { DirectOwner, SpatialIdentity, CanonicalPermission };

Mode parse_mode(const std::string& value) {
  if (value == "direct-owner") return Mode::DirectOwner;
  if (value == "spatial-identity") return Mode::SpatialIdentity;
  if (value == "canonical-permission") return Mode::CanonicalPermission;
  throw std::runtime_error("observer_mode_invalid");
}

bool valid_nonce(const std::string& nonce) {
  return nonce.size() == 32 && nonce.find_first_not_of("0123456789abcdef") == std::string::npos;
}

class Json {
 public:
  void append(const std::string& text) {
    if (text.size() > kMaximumSampleBytes - value_.size()) {
      throw std::runtime_error("observer_sample_too_large");
    }
    value_ += text;
  }
  void quote(const std::string& text) {
    append("\"");
    constexpr char hex[] = "0123456789abcdef";
    for (unsigned char byte : text) {
      if (byte == '"' || byte == '\\') {
        append(std::string("\\") + static_cast<char>(byte));
      } else if (byte < 0x20) {
        std::string escaped = "\\u00";
        escaped += hex[byte >> 4];
        escaped += hex[byte & 15];
        append(escaped);
      } else {
        append(std::string(1, static_cast<char>(byte)));
      }
    }
    append("\"");
  }
  const std::string& value() const { return value_; }
 private:
  std::string value_;
};

std::string gid_hex(const rmw_gid_t& gid) {
  static_assert(RMW_GID_STORAGE_SIZE == 24, "review publisher GID wire size");
  constexpr char hex[] = "0123456789abcdef";
  std::string value;
  bool nonzero = false;
  for (const auto byte : gid.data) {
    nonzero = nonzero || byte != 0;
    value += hex[byte >> 4];
    value += hex[byte & 15];
  }
  if (!nonzero) throw std::runtime_error("observer_publisher_gid_unavailable");
  return value;
}

int64_t monotonic_ns() {
  timespec value{};
  if (::clock_gettime(CLOCK_MONOTONIC, &value) != 0) {
    throw std::runtime_error("observer_monotonic_clock_unavailable");
  }
  return static_cast<int64_t>(value.tv_sec) * 1000000000 + value.tv_nsec;
}

std::string sample_json(const Message& message, const diagnostic_msgs::msg::DiagnosticStatus& row,
                        const rmw_gid_t& gid, int64_t ros_ns, int64_t steady_ns) {
  Json out;
  out.append("{\"name\":"); out.quote(row.name);
  out.append(",\"hardware_id\":"); out.quote(row.hardware_id);
  out.append(",\"level\":" + std::to_string(row.level) + ",\"values\":[");
  bool first = true;
  for (const auto& pair : row.values) {
    if (!first) out.append(",");
    first = false;
    out.append("["); out.quote(pair.key); out.append(","); out.quote(pair.value); out.append("]");
  }
  out.append("],\"publisher_gid\":"); out.quote(gid_hex(gid));
  const auto stamp = static_cast<int64_t>(message.header.stamp.sec) * 1000000000 +
                     message.header.stamp.nanosec;
  out.append(",\"stamp_ns\":" + std::to_string(stamp) +
             ",\"receipt_ros_ns\":" + std::to_string(ros_ns) +
             ",\"receipt_steady_ns\":" + std::to_string(steady_ns) + "}");
  return out.value();
}

struct Latest {
  std::string sample;
  std::string error;
};

class Observer final : public rclcpp::Node {
 public:
  explicit Observer(Mode mode, rclcpp::Context::SharedPtr context = nullptr)
      : Node("sunhorizon_attended_diagnostic_observer", options(context)), mode_(mode) {
    const char* topic = mode != Mode::SpatialIdentity ? "/diagnostics" :
        "/sunhorizon/obstacle_coverage/spatial_identity_v1";
    const auto qos = mode == Mode::CanonicalPermission ?
        rclcpp::QoS(1).best_effort().durability_volatile() : rclcpp::QoS(1).reliable().transient_local();
    subscription_ = create_subscription<Message>(topic, qos,
        [this](Message::ConstSharedPtr message, const rclcpp::MessageInfo& info) {
          // Capture at this callback, never when Python later asks for latest.
          try {
            const auto ros_ns = get_clock()->now().nanoseconds();
            const auto steady_ns = monotonic_ns();
            const diagnostic_msgs::msg::DiagnosticStatus* selected = nullptr;
            if (mode_ == Mode::SpatialIdentity) {
              if (message->status.size() != 1) throw std::runtime_error("observer_status_count");
              selected = &message->status.front();
            } else {
              for (const auto& row : message->status) {
                const char* target = mode_ == Mode::CanonicalPermission ?
                    "sunhorizon_nav2_direct/canonical_motion_permission" : kDirectRow;
                if (row.name != target) continue;
                if (selected) throw std::runtime_error("observer_duplicate_target_row");
                selected = &row;
              }
              if (!selected) return;  // Unrelated diagnostics do not replace this source.
            }
            auto sample = sample_json(*message, *selected, info.get_rmw_message_info().publisher_gid,
                                      ros_ns, steady_ns);
            latest_ = Latest{std::move(sample), {}};
          } catch (const std::exception& error) {
            // A malformed newer target invalidates the previous good sample.
            latest_ = Latest{{}, error.what()};
          }
        });
  }
  const Latest& latest() const { return latest_; }
 private:
  static rclcpp::NodeOptions options(const rclcpp::Context::SharedPtr& context) {
    rclcpp::NodeOptions result;
    result.enable_rosout(false).start_parameter_services(false)
        .start_parameter_event_publisher(false).use_global_arguments(false);
    if (context) result.context(context);
    return result;
  }
  Mode mode_;
  Latest latest_;
  rclcpp::Subscription<Message>::SharedPtr subscription_;
};

std::string envelope(const std::string& nonce, uint64_t id, const std::string& kind) {
  return "{\"schema\":1,\"nonce\":\"" + nonce + "\",\"id\":" + std::to_string(id) +
         ",\"kind\":\"" + kind + "\"";
}

// Deliberately one canonical GET encoding, not a general JSON command API.
uint64_t request_id(const std::string& line, const std::string& nonce, uint64_t previous) {
  const std::string prefix = "{\"schema\":1,\"nonce\":\"" + nonce + "\",\"id\":";
  const std::string suffix = ",\"kind\":\"get\"}";
  if (line.size() > 256 || line.compare(0, prefix.size(), prefix) != 0 ||
      line.size() <= prefix.size() + suffix.size() ||
      line.compare(line.size() - suffix.size(), suffix.size(), suffix) != 0) {
    throw std::runtime_error("observer_request_invalid");
  }
  const auto digits = line.substr(prefix.size(), line.size() - prefix.size() - suffix.size());
  if (digits.size() > 16 || digits.front() == '0' ||
      digits.find_first_not_of("0123456789") != std::string::npos) {
    throw std::runtime_error("observer_request_id_invalid");
  }
  const auto id = std::stoull(digits);
  if (id != previous + 1) throw std::runtime_error("observer_request_id_sequence");
  return id;
}

std::string report(const std::string& nonce, uint64_t id, const Latest& latest) {
  Json error;
  error.quote(latest.error);
  const auto value = envelope(nonce, id, "report") + ",\"sample\":" +
      (latest.sample.empty() ? "null" : latest.sample) + ",\"error\":" + error.value() + "}\n";
  if (value.size() > kMaximumLineBytes) throw std::runtime_error("observer_reply_too_large");
  return value;
}

void nonblocking(int fd) {
  const auto flags = ::fcntl(fd, F_GETFL);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) < 0) {
    throw std::runtime_error("observer_pipe_setup_failed");
  }
}

void write_reply(const std::string& line) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  std::size_t offset = 0;
  while (offset < line.size()) {
    if (std::chrono::steady_clock::now() >= deadline) throw std::runtime_error("observer_write_timeout");
    pollfd fd{STDOUT_FILENO, POLLOUT, 0};
    const auto ready = ::poll(&fd, 1, 10);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0 || (fd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
      throw std::runtime_error("observer_output_closed");
    }
    if (!ready) continue;
    const auto count = ::write(STDOUT_FILENO, line.data() + offset, line.size() - offset);
    if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (count <= 0) throw std::runtime_error("observer_write_failed");
    offset += static_cast<std::size_t>(count);
  }
}

void serve(const std::shared_ptr<Observer>& node, const std::string& nonce) {
  nonblocking(STDIN_FILENO);
  nonblocking(STDOUT_FILENO);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  write_reply(envelope(nonce, 0, "ready") + "}\n");
  std::string pending;
  uint64_t previous = 0;
  while (rclcpp::ok()) {
    executor.spin_some(std::chrono::milliseconds(2));
    pollfd fd{STDIN_FILENO, POLLIN, 0};
    const auto ready = ::poll(&fd, 1, 5);
    if (ready < 0 && errno == EINTR) continue;
    if (ready < 0 || (fd.revents & (POLLERR | POLLNVAL))) throw std::runtime_error("observer_input_failed");
    if (!ready) continue;
    char bytes[256];
    const auto count = ::read(STDIN_FILENO, bytes, sizeof(bytes));
    if (count < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (count < 0) throw std::runtime_error("observer_read_failed");
    if (count == 0) {
      if (!pending.empty()) throw std::runtime_error("observer_request_truncated");
      return;  // Parent closed its private stdin; no other process is signalled.
    }
    pending.append(bytes, static_cast<std::size_t>(count));
    const auto newline = pending.find('\n');
    if (newline != std::string::npos) {
      const auto id = request_id(pending.substr(0, newline), nonce, previous);
      // No pipelined commands: there is exactly one outstanding parent request.
      if (newline + 1 != pending.size()) throw std::runtime_error("observer_request_pipeline");
      write_reply(report(nonce, id, node->latest()));
      previous = id;
      pending.clear();
    } else if (pending.size() > 256) {
      throw std::runtime_error("observer_request_too_large");
    }
  }
  throw std::runtime_error("observer_ros_shutdown");
}
}  // namespace sunhorizon_navigation_runtime::diagnostic_observer

#ifndef SUNHORIZON_DIAGNOSTIC_OBSERVER_NO_MAIN
int main(int argc, char** argv) {
  namespace observer = sunhorizon_navigation_runtime::diagnostic_observer;
  try {
    if (argc != 5 || std::string(argv[1]) != "--mode" || std::string(argv[3]) != "--nonce" ||
        !observer::valid_nonce(argv[4])) throw std::runtime_error("observer_arguments_invalid");
    const auto mode = observer::parse_mode(argv[2]);
    std::signal(SIGPIPE, SIG_IGN);
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<observer::Observer>(mode);
    observer::serve(node, argv[4]);
    node.reset();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (rclcpp::ok()) rclcpp::shutdown();
    return 1;
  }
}
#endif
