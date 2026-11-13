// Exercise the actual observer callback and serializer, including real DDS GIDs.
#define SUNHORIZON_DIAGNOSTIC_OBSERVER_NO_MAIN
#include "../src/diagnostic_identity_observer.cpp"

#include <functional>
#include <thread>

namespace observer = sunhorizon_navigation_runtime::diagnostic_observer;

void require(bool value, const char* reason) {
  if (!value) throw std::runtime_error(reason);  // Release builds retain checks.
}

template <class Function>
void rejects(Function operation) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  require(rejected, "expected rejection");
}

void protocol_and_serialization() {
  const std::string nonce(32, 'a');
  require(observer::valid_nonce(nonce), "valid nonce rejected");
  require(!observer::valid_nonce(std::string(32, 'g')), "invalid nonce accepted");
  const auto request = observer::envelope(nonce, 1, "get") + "}";
  require(observer::request_id(request, nonce, 0) == 1, "first GET rejected");
  rejects([&] { observer::request_id(request, nonce, 1); });
  rejects([&] { observer::request_id(request, std::string(32, 'b'), 0); });
  rejects([&] { observer::request_id(request + "extra", nonce, 0); });
  rejects([&] { observer::parse_mode("arbitrary-topic"); });
  require(observer::report(nonce, 1, {}) == observer::envelope(nonce, 1, "report") +
          ",\"sample\":null,\"error\":\"\"}\n", "missing is not explicit");
  observer::Json quoted;
  quoted.quote("a\"\\\n");
  require(quoted.value() == "\"a\\\"\\\\\\u000a\"", "JSON escaping changed");
  observer::Message message;
  message.header.stamp.sec = 12;
  message.header.stamp.nanosec = 34;
  diagnostic_msgs::msg::DiagnosticStatus row;
  row.name = observer::kDirectRow;
  row.hardware_id = "epoch";
  diagnostic_msgs::msg::KeyValue pair;
  pair.key = "duplicate";
  pair.value = "original";
  row.values = {pair, pair};
  rmw_gid_t gid{};
  rejects([&] { observer::sample_json(message, row, gid, 100, 200); });
  gid.data[0] = 0xab;
  const auto sample = observer::sample_json(message, row, gid, 100, 200);
  require(sample.find("[\"duplicate\",\"original\"],[\"duplicate\",\"original\"]") !=
          std::string::npos, "duplicate payload keys were silently collapsed");
  require(sample.find("\"stamp_ns\":12000000034,\"receipt_ros_ns\":100,\"receipt_steady_ns\":200") !=
          std::string::npos, "original clocks not retained");
  row.values.front().value.assign(observer::kMaximumSampleBytes, 'x');
  rejects([&] { observer::sample_json(message, row, gid, 100, 200); });
}

void spin_until(rclcpp::executors::SingleThreadedExecutor& executor,
                const std::function<bool()>& done) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!done() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  require(done(), "DDS test deadline");
}

void real_message_info(observer::Mode mode, const rclcpp::Context::SharedPtr& context) {
  auto sink = std::make_shared<observer::Observer>(mode, context);
  auto source = std::make_shared<rclcpp::Node>("diagnostic_observer_test_source",
      rclcpp::NodeOptions().context(context).enable_rosout(false));
  const char* topic = mode != observer::Mode::SpatialIdentity ? "/diagnostics" :
      "/sunhorizon/obstacle_coverage/spatial_identity_v1";
  const auto qos = mode == observer::Mode::CanonicalPermission ?
      rclcpp::QoS(1).best_effort().durability_volatile() : rclcpp::QoS(1).reliable().transient_local();
  auto publisher = source->create_publisher<observer::Message>(topic, qos);
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(sink);
  executor.add_node(source);
  spin_until(executor, [&] { return publisher->get_subscription_count() == 1; });
  observer::Message message;
  message.header.stamp.sec = 123;
  diagnostic_msgs::msg::DiagnosticStatus row;
  row.name = mode == observer::Mode::DirectOwner ? observer::kDirectRow :
      (mode == observer::Mode::CanonicalPermission ? "sunhorizon_nav2_direct/canonical_motion_permission" : "identity");
  row.hardware_id = "original-epoch";
  message.status = {row};
  publisher->publish(message);
  spin_until(executor, [&] { return !sink->latest().sample.empty(); });
  const auto first = sink->latest().sample;
  const auto real_gid = observer::gid_hex(publisher->get_gid());
  require(first.find("\"publisher_gid\":\"" + real_gid + "\"") != std::string::npos,
          "callback did not preserve actual MessageInfo publisher GID");
  require(first.find("\"stamp_ns\":123000000000") != std::string::npos,
          "callback changed original header");
  require(first.find("\"receipt_ros_ns\":") != std::string::npos &&
          first.find("\"receipt_steady_ns\":") != std::string::npos,
          "callback receipts missing");
  observer::report(std::string(32, 'a'), 1, sink->latest());
  require(sink->latest().sample == first, "GET refreshed callback clocks");
  if (mode != observer::Mode::SpatialIdentity) {
    message.status.front().name = "unrelated";
    publisher->publish(message);
    for (int i = 0; i < 30; ++i) {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    require(sink->latest().sample == first, "unrelated row replaced source");
    message.status.front() = row;
  }
  message.status.push_back(row);
  publisher->publish(message);
  spin_until(executor, [&] { return !sink->latest().error.empty(); });
  require(sink->latest().sample.empty(), "malformed newer sample replayed last good source");
  row.values.resize(1);
  row.values.front().value.assign(observer::kMaximumSampleBytes, 'x');
  message.status = {row};
  publisher->publish(message);
  spin_until(executor, [&] { return sink->latest().error == "observer_sample_too_large"; });
  require(sink->latest().sample.empty(), "oversized sample retained stale good source");
  executor.remove_node(sink);
  executor.remove_node(source);
}

int main() {
  auto context = std::make_shared<rclcpp::Context>();
  try {
    protocol_and_serialization();
    rclcpp::InitOptions options;
    options.set_domain_id(100 + static_cast<std::size_t>(::getpid()) % 100);
    context->init(0, nullptr, options);
    real_message_info(observer::Mode::DirectOwner, context);
    real_message_info(observer::Mode::SpatialIdentity, context);
    real_message_info(observer::Mode::CanonicalPermission, context);
    context->shutdown("test complete");
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    if (context->is_valid()) context->shutdown("test failure");
    return 1;
  }
}
