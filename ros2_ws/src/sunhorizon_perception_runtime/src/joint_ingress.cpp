#include "sunhorizon_perception_runtime/joint_ingress.hpp"
#include "sunhorizon_perception_runtime/joint_startup_contract.hpp"
#include <tf2/LinearMath/Quaternion.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <ctime>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <cstdlib>
#include <cerrno>
#include <iomanip>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>
#include <nlohmann/json.hpp>
#include <openssl/sha.h>

#include <rmw/types.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

extern char** environ;

namespace sunhorizon_perception_runtime {
namespace {
namespace platform = sunhorizon_platform_adapter_casbot02;

int64_t steady_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

int64_t diagnostic_thread_cpu_ns() noexcept {
  timespec value{};
  if (::clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) return -1;
  return int64_t(value.tv_sec) * 1000000000LL + value.tv_nsec;
}

JointIngressClockPoint ingress_clock_point() {
  return {steady_ns(), diagnostic_thread_cpu_ns()};
}

std::string gid_hex(const uint8_t* bytes, std::size_t size) {
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t i = 0; i < size; ++i)
    output << std::setw(2) << static_cast<unsigned int>(bytes[i]);
  return output.str();
}
std::string gid_hex(const rmw_gid_t& value) {
  return gid_hex(value.data, RMW_GID_STORAGE_SIZE);
}
template <std::size_t Size>
std::string gid_hex(const std::array<uint8_t, Size>& value) {
  return gid_hex(value.data(), value.size());
}

std::string pinned_contract_bytes(const std::string& filename, const std::string& expected_sha) {
  const std::regex digest("[0-9a-f]{64}");
  const std::filesystem::path path(filename);
  if (!std::regex_match(expected_sha, digest) || !path.is_absolute() ||
      std::filesystem::canonical(path) != path ||
      !std::filesystem::is_regular_file(std::filesystem::symlink_status(path)))
    throw std::invalid_argument("joint_startup_asset_not_canonical_or_sha_invalid");
  const auto size = std::filesystem::file_size(path);
  const auto modified = std::filesystem::last_write_time(path);
  if (size == 0 || size > 16*1024*1024)
    throw std::invalid_argument("joint_startup_asset_size");
  std::string bytes(static_cast<std::size_t>(size), '\0');
  std::ifstream input(path, std::ios::binary);
  if (!input.read(bytes.data(), static_cast<std::streamsize>(bytes.size())) ||
      input.peek() != std::char_traits<char>::eof() || std::filesystem::file_size(path) != size ||
      std::filesystem::last_write_time(path) != modified)
    throw std::runtime_error("joint_startup_asset_changed_while_reading");
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), hash);
  if (gid_hex(hash, SHA256_DIGEST_LENGTH) != expected_sha)
    throw std::runtime_error("joint_startup_actual_file_sha_mismatch");
  return bytes;
}

std::string run_contract_validator(const std::string& arguments) {
  // Fixed code plus argv JSON; no shell, command interpolation or user-selected
  // executable. Existing contract validators invoke the reviewed ssh-keygen.
  static const std::string program = R"py(
import json, runpy, sys
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
path = Path(get_package_share_directory('sunhorizon_perception_runtime')) / 'contract' / 'joint_startup_contract.py'
module = runpy.run_path(str(path))
print(json.dumps(module['validate_startup'](json.loads(sys.argv[1])), separators=(',', ':')))
)py";
  int descriptors[2];
  if (pipe(descriptors) != 0) throw std::runtime_error("joint_startup_validator_pipe_failed");
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attributes;
  posix_spawn_file_actions_init(&actions);
  posix_spawnattr_init(&attributes);
  posix_spawn_file_actions_addclose(&actions, descriptors[0]);
  posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDOUT_FILENO);
  posix_spawn_file_actions_adddup2(&actions, descriptors[1], STDERR_FILENO);
  posix_spawn_file_actions_addclose(&actions, descriptors[1]);
  posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
  posix_spawnattr_setpgroup(&attributes, 0);
  std::vector<std::string> values{"/usr/bin/python3", "-B", "-c", program, arguments};
  std::vector<char*> argv;
  for (auto& value : values) argv.push_back(value.data());
  argv.push_back(nullptr);
  pid_t process = 0;
  const int result = posix_spawn(&process, "/usr/bin/python3", &actions, &attributes, argv.data(), environ);
  posix_spawn_file_actions_destroy(&actions);
  posix_spawnattr_destroy(&attributes);
  close(descriptors[1]);
  if (result != 0) {close(descriptors[0]); throw std::runtime_error("joint_startup_validator_spawn_failed");}
  const int descriptor_flags=fcntl(descriptors[0], F_GETFL);
  if (descriptor_flags < 0 || fcntl(descriptors[0], F_SETFL, descriptor_flags | O_NONBLOCK) < 0) {
    kill(-process, SIGKILL);
    int ignored=0;while (waitpid(process, &ignored, 0)<0 && errno==EINTR) {}
    close(descriptors[0]);
    throw std::runtime_error("joint_startup_validator_nonblocking_pipe_failed");
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  std::string output;
  int status = 0;
  bool done = false;
  while (true) {
    char bytes[4096];
    ssize_t count = 0;
    while ((count = read(descriptors[0], bytes, sizeof(bytes))) > 0) {
      output.append(bytes, static_cast<std::size_t>(count));
      if (output.size() > 16*1024*1024) break;
    }
    if (!done) {
      const auto waited = waitpid(process, &status, WNOHANG);
      done = waited == process;
      if (waited < 0 && errno != EINTR) {
        kill(-process, SIGKILL); close(descriptors[0]);
        throw std::runtime_error("joint_startup_validator_wait_failed");
      }
    }
    if (done && count == 0) break;
    if (output.size() > 16*1024*1024 || std::chrono::steady_clock::now() >= deadline) {
      kill(-process, SIGKILL);
      if (!done) while (waitpid(process, &status, 0) < 0 && errno == EINTR) {}
      close(descriptors[0]);
      throw std::runtime_error("joint_startup_validator_timeout_or_output_bound");
    }
    pollfd ready{descriptors[0], POLLIN, 0};
    (void)poll(&ready, 1, 20);
  }
  close(descriptors[0]);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0)
    throw std::runtime_error("joint_startup_signed_contract_validation_failed:" + output.substr(0, 4096));
  return output;
}

std::string startup_parameter(rclcpp::Node& node, const std::string& name,
                              const char* environment_name = nullptr) {
  if (node.has_parameter(name)) return node.get_parameter(name).as_string();
  const char* value = environment_name ? std::getenv(environment_name) : nullptr;
  rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
  return node.declare_parameter(name, std::string(value ? value : ""), descriptor);
}
}  // namespace

JointStartupOptions load_joint_startup_options(rclcpp::Node& node) {
  JointStartupOptions options;
  options.robot_serial = startup_parameter(node, "joint_contract_robot_serial", "SUNHORIZON_ROBOT_SERIAL");
  options.release_sha = startup_parameter(node, "joint_contract_release_sha", "SUNHORIZON_PLATFORM_EVIDENCE_RELEASE_SHA");
  options.source_receipt_path = startup_parameter(node, "source_receipt_path");
  options.source_receipt_sha256 = startup_parameter(node, "source_receipt_sha256");
  options.source_evidence_sha256 = startup_parameter(node, "source_evidence_sha256");
  options.mapping_receipt_path = startup_parameter(node, "leg_joint_name_mapping_receipt_path");
  options.mapping_receipt_sha256 = startup_parameter(node, "leg_joint_name_mapping_receipt_sha256");
  options.source_review_authority_sha256 = startup_parameter(node, "joint_source_review_authority_sha256",
      "SUNHORIZON_LIVE_JOINT_REVIEW_AUTHORITY_SHA256");
  options.mapping_review_authority_sha256 = startup_parameter(node, "joint_mapping_review_authority_sha256",
      "SUNHORIZON_LEG_MAPPING_REVIEW_AUTHORITY_SHA256");
  options.tf_mode = startup_parameter(node, "kinematics_tf_mode");
  options.attended_usage = startup_parameter(node,"kinematics_attended_usage");
  options.attended_tf_asset_path = startup_parameter(node,"kinematics_attended_sensor_tf_asset_path");
  options.attended_tf_asset_sha256 = startup_parameter(node,"kinematics_attended_sensor_tf_asset_sha256");
  options.attended_tf_receipt_path = startup_parameter(node,"kinematics_attended_sensor_tf_receipt_path");
  options.attended_tf_receipt_sha256 = startup_parameter(node,"kinematics_attended_sensor_tf_receipt_sha256");
  options.production_tf_receipt_path = startup_parameter(node, "kinematics_production_sensor_tf_receipt_path");
  options.production_tf_receipt_sha256 = startup_parameter(node, "kinematics_production_sensor_tf_receipt_sha256");
  return options;
}

ValidatedJointStartup validate_joint_startup(const JointStartupOptions& options,
    const std::string& legacy_urdf_path, const std::string& legacy_urdf_sha256) {
  if (options.tf_mode != "attended_global_tf" && options.tf_mode != "attended_direct" && options.tf_mode != "attended_measured_direct" && options.tf_mode != "production_direct")
    throw std::invalid_argument("joint_startup_explicit_tf_mode_required");
  if (options.tf_mode != "production_direct" &&
      (!options.production_tf_receipt_path.empty() || !options.production_tf_receipt_sha256.empty()))
    throw std::invalid_argument("joint_startup_attended_mode_must_not_claim_production_acceptance");
  const auto source = pinned_contract_bytes(options.source_receipt_path, options.source_receipt_sha256);
  const auto mapping = pinned_contract_bytes(options.mapping_receipt_path, options.mapping_receipt_sha256);
  nlohmann::json arguments = {
      {"robot_serial", options.robot_serial}, {"release_sha", options.release_sha},
      {"source_receipt_path", options.source_receipt_path},
      {"source_receipt_sha256", options.source_receipt_sha256},
      {"source_evidence_sha256", options.source_evidence_sha256},
      {"mapping_receipt_path", options.mapping_receipt_path},
      {"mapping_receipt_sha256", options.mapping_receipt_sha256},
      {"source_review_authority_sha256", options.source_review_authority_sha256},
      {"mapping_review_authority_sha256", options.mapping_review_authority_sha256},
      {"tf_mode", options.tf_mode}, {"attended_usage", options.attended_usage},
      {"attended_tf_asset_path", options.attended_tf_asset_path},
      {"attended_tf_asset_sha256", options.attended_tf_asset_sha256},
      {"attended_tf_receipt_path", options.attended_tf_receipt_path},
      {"attended_tf_receipt_sha256", options.attended_tf_receipt_sha256},
      {"production_tf_receipt_path", options.production_tf_receipt_path},
      {"production_tf_receipt_sha256", options.production_tf_receipt_sha256},
      {"legacy_urdf_path", legacy_urdf_path}, {"legacy_urdf_sha256", legacy_urdf_sha256}};
  if (!legacy_urdf_path.empty()) {
    (void)pinned_contract_bytes(legacy_urdf_path, legacy_urdf_sha256);
  }
  if (options.tf_mode == "production_direct") {
    (void)pinned_contract_bytes(options.production_tf_receipt_path, options.production_tf_receipt_sha256);
  }
  if (options.tf_mode == "attended_direct" || options.tf_mode == "attended_measured_direct") {
    (void)pinned_contract_bytes(options.attended_tf_asset_path,options.attended_tf_asset_sha256);
    (void)pinned_contract_bytes(options.attended_tf_receipt_path,options.attended_tf_receipt_sha256);
  }
  const auto document = nlohmann::json::parse(run_contract_validator(arguments.dump()));
  if (pinned_contract_bytes(options.source_receipt_path, options.source_receipt_sha256) != source ||
      pinned_contract_bytes(options.mapping_receipt_path, options.mapping_receipt_sha256) != mapping)
    throw std::runtime_error("joint_startup_receipt_changed_after_validation");
  ValidatedJointStartup result;
  if (options.tf_mode == "attended_direct" || options.tf_mode == "attended_measured_direct") {
    for (const auto& edge : document.at("attended_static_edges")) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.frame_id=edge.at("parent").get<std::string>();
      transform.child_frame_id=edge.at("child").get<std::string>();
      const auto xyz=edge.at("xyz_m").get<std::vector<double>>();
      const auto rpy=edge.at("rpy_rad").get<std::vector<double>>();
      if(xyz.size()!=3 || rpy.size()!=3)throw std::runtime_error("attended_static_edge_shape_invalid");
      transform.transform.translation.x=xyz[0];transform.transform.translation.y=xyz[1];transform.transform.translation.z=xyz[2];
      tf2::Quaternion q;q.setRPY(rpy[0],rpy[1],rpy[2]);
      transform.transform.rotation.x=q.x();transform.transform.rotation.y=q.y();
      transform.transform.rotation.z=q.z();transform.transform.rotation.w=q.w();
      result.attended_static_edges.push_back(std::move(transform));
    }
    if(result.attended_static_edges.size()!=8)throw std::runtime_error("attended_static_edge_count_invalid");
  }
  const auto& input = document.at("source");
  result.expected_source_names = input.at("expected_source_joint_names").get<std::vector<std::string>>();
  result.selected_source_names = input.at("selected_source_joint_names").get<std::vector<std::string>>();
  result.selected_source_indices = input.at("selected_source_joint_indices").get<std::vector<std::size_t>>();
  result.output_joint_names = document.at("output_joint_names").get<std::vector<std::string>>();
  if (!legacy_urdf_path.empty()) {
    result.merged_urdf_xml = document.at("merged_urdf_xml").get<std::string>();
    result.merged_urdf_sha256 = document.at("merged_urdf_sha256").get<std::string>();
    result.leg_target_frames = document.at("leg_target_frames").get<std::vector<std::string>>();
    result.waist_source_joint_name = "waist_yaw_joint";
    result.waist_output_joint_name = document.at("joint_name_aliases").at("waist_yaw_joint").get<std::string>();
  }
  return result;
}

struct JointIngress::Impl {
  struct Owner {
    platform::LiveJointStateOwnerStatus status;
    std::string endpoint_gid;
  };
  struct Proof {
    sensor_msgs::msg::JointState message;
    int64_t source_stamp_ns;
    int64_t receipt_steady_ns;
  };

  Impl(JointIngress& value, std::shared_ptr<JointKinematics> model)
      : node(value), kinematics(std::move(model)) {
    if (!kinematics) throw std::invalid_argument("joint_ingress_kinematics_required");
    const bool contract_accepted = node.declare_parameter("source_contract_accepted", false);
    source_topic = node.declare_parameter("source_topic", "/joint_states");
    const auto source_type = node.declare_parameter("source_type", "sensor_msgs/msg/JointState");
    owner_name = node.declare_parameter("source_owner_name", "");
    owner_namespace = node.declare_parameter("source_owner_namespace", "");
    source_owner_fqn = (owner_namespace == "/" ? "/" : owner_namespace + "/") + owner_name;
    const auto runtime_binding = node.declare_parameter("runtime_binding", "");
    const auto output_topic = node.declare_parameter(
        "output_topic", "/sunhorizon/platform/proof/live_joint_states");
    const auto reason_topic = node.declare_parameter(
        "reason_topic", "/sunhorizon/platform/proof/live_joint_state_reason");
    const auto source_names = node.declare_parameter<std::vector<std::string>>(
        "expected_source_joint_names", std::vector<std::string>{});
    const auto selected_names = node.declare_parameter<std::vector<std::string>>(
        "selected_source_joint_names", std::vector<std::string>{});
    const auto indices_parameter = node.declare_parameter<std::vector<int64_t>>(
        "selected_source_joint_indices", std::vector<int64_t>{});
    const auto output_names = node.declare_parameter<std::vector<std::string>>(
        "output_joint_names", std::vector<std::string>{});
    const auto unit = node.declare_parameter("position_unit", "");
    const auto value_transform = node.declare_parameter("value_transform", "");
    rcl_interfaces::msg::ParameterDescriptor deadline_descriptor;
    deadline_descriptor.read_only = true;
    const bool attended_deadlines = node.declare_parameter(
        "attended_doubled_deadlines", false, deadline_descriptor);
    const double maximum_age_sec = node.declare_parameter("maximum_source_age_sec", 0.25);
    const double future_tolerance_sec = node.declare_parameter("source_future_tolerance_sec", 0.05);
    const double discovery_grace_sec = node.declare_parameter("source_owner_discovery_grace_sec", 12.0);
    receipt.source_receipt_path = node.declare_parameter("source_receipt_path", "");
    receipt.source_receipt_sha256 = node.declare_parameter("source_receipt_sha256", "");
    receipt.source_evidence_sha256 = node.declare_parameter("source_evidence_sha256", "");
    const std::regex sha256("[0-9a-f]{64}");
    // Identical accepted parameter domain to the existing live bridge. The
    // actual signed receipts are additionally revalidated below in this owner.
    if (!contract_accepted || source_topic != "/joint_states" ||
        source_type != "sensor_msgs/msg/JointState" || owner_name != "hl_orin_core_msg" ||
        owner_namespace != "/" ||
        runtime_binding != "unique_current_endpoint_name_type_and_message_gid" ||
        output_topic != "/sunhorizon/platform/proof/live_joint_states" || unit != "rad" ||
        value_transform != "identity" || !std::isfinite(maximum_age_sec) ||
        maximum_age_sec <= 0.0 || maximum_age_sec > (attended_deadlines ? 0.5 : 0.25) ||
        !std::isfinite(future_tolerance_sec) || future_tolerance_sec < 0.0 ||
        future_tolerance_sec > 0.05 || !std::isfinite(discovery_grace_sec) ||
        discovery_grace_sec <= 0.0 || discovery_grace_sec > 12.0 ||
        receipt.source_receipt_path.empty() ||
        !std::filesystem::path(receipt.source_receipt_path).is_absolute() ||
        !std::regex_match(receipt.source_receipt_sha256, sha256) ||
        !std::regex_match(receipt.source_evidence_sha256, sha256))
      throw std::runtime_error("live joint state source contract invalid");
    maximum_age_ns = static_cast<int64_t>(maximum_age_sec * 1e9);
    future_tolerance_ns = static_cast<int64_t>(future_tolerance_sec * 1e9);
    if (kinematics->options().expected_joint_names != output_names ||
        kinematics->options().maximum_age_ns > maximum_age_ns ||
        kinematics->options().future_tolerance_ns > future_tolerance_ns)
      throw std::runtime_error("joint_ingress_kinematics_contract_mismatch");
    std::vector<std::size_t> indices;
    for (const auto index : indices_parameter) {
      if (index < 0 || index >= 49)
        throw std::runtime_error("live joint state selected index invalid");
      indices.push_back(static_cast<std::size_t>(index));
    }
    gate = std::make_unique<platform::LiveJointStateBridgeGate>(source_names, selected_names,
        indices, output_names, unit, value_transform, maximum_age_ns, future_tolerance_ns);
    const auto startup_options = load_joint_startup_options(node);
    const auto validated = validate_joint_startup(startup_options);
    const auto& model_contract = kinematics->options();
    if (validated.expected_source_names != source_names ||
        validated.selected_source_names != selected_names ||
        validated.selected_source_indices != indices || validated.output_joint_names != output_names ||
        model_contract.source_receipt_sha256 != startup_options.source_receipt_sha256 ||
        model_contract.mapping_receipt_sha256 != startup_options.mapping_receipt_sha256 ||
        model_contract.tf_mode != startup_options.tf_mode ||
        model_contract.attended_usage != startup_options.attended_usage ||
        model_contract.attended_tf_asset_sha256 != startup_options.attended_tf_asset_sha256 ||
        model_contract.attended_tf_receipt_sha256 != startup_options.attended_tf_receipt_sha256 ||
        (startup_options.tf_mode == "production_direct" && model_contract.production_tf_receipt_sha256.empty()) ||
        model_contract.production_tf_receipt_sha256 != startup_options.production_tf_receipt_sha256)
      throw std::runtime_error("joint_ingress_actual_receipt_or_model_contract_mismatch");
    discovery_gate = std::make_unique<platform::LiveJointStateDiscoveryGate>(
        static_cast<int64_t>(discovery_grace_sec * 1e9));
    discovery_started_ns = steady_ns();
    rclcpp::SensorDataQoS qos;
    qos.keep_last(1);
    publisher = node.create_publisher<sensor_msgs::msg::JointState>(output_topic, qos);
    reason_publisher = node.create_publisher<std_msgs::msg::String>(reason_topic, 10);
    subscription = node.create_subscription<sensor_msgs::msg::JointState>(source_topic, qos,
        [this](sensor_msgs::msg::JointState::ConstSharedPtr message,
               const rclcpp::MessageInfo& info) {on_joint_state(std::move(message), info);});
    discovery_timer = node.create_wall_timer(std::chrono::milliseconds(50), [this]() {
      const int64_t receipt_ros = node.get_clock()->now().nanoseconds();
      const int64_t receipt_steady = steady_ns();
      (void)discovery_ready(owner(), receipt_ros, receipt_steady);
    });
    egress_thread = std::thread([this]() {egress();});
  }

  ~Impl() {
    {
      std::lock_guard<std::mutex> lock(egress_mutex);
      stopping = true;
      pending_proof.reset();
    }
    egress_condition.notify_one();
    if (egress_thread.joinable()) egress_thread.join();
  }

  Owner owner() const {
    const auto endpoints = node.get_publishers_info_by_topic(source_topic);
    const platform::LiveJointStatePublisherIdentity expected{
        owner_name, owner_namespace, "sensor_msgs/msg/JointState", ""};
    platform::LiveJointStatePublisherIdentity observed;
    if (endpoints.size() == 1) {
      const auto& endpoint = endpoints.front();
      observed = {endpoint.node_name(), endpoint.node_namespace(), endpoint.topic_type(),
                  gid_hex(endpoint.endpoint_gid())};
    }
    return {platform::live_joint_source_owner_status(endpoints.size(), expected, observed),
            observed.endpoint_gid};
  }

  void reject(const platform::LiveJointStateDecision& decision,
              int64_t receipt_ros, int64_t receipt_steady) {
    (void)kinematics->ingest(decision, receipt_ros, receipt_steady);
    {
      std::lock_guard<std::mutex> lock(egress_mutex);
      pending_proof.reset();
      if (decision.fault_latched && !fault_reported) {
        fault_reported = true;
        pending_reason = decision.reason;
      }
    }
    egress_condition.notify_one();
  }

  bool discovery_ready(const Owner& current, int64_t receipt_ros, int64_t receipt_steady) {
    const auto decision = discovery_gate->evaluate(current.status,
                                                  receipt_steady - discovery_started_ns);
    if (decision.accepted) {
      discovery_timeout_reported = false;
      if (discovery_timer) discovery_timer->cancel();
      return true;
    }
    reject({false, decision.fault_latched, decision.reason, 0, {}, {}},
           receipt_ros, receipt_steady);
    if (!decision.fault_latched &&
        decision.reason == "live_joint_source_owner_discovery_timeout" &&
        !discovery_timeout_reported) {
      discovery_timeout_reported = true;
      {
        std::lock_guard<std::mutex> lock(egress_mutex);
        pending_reason = decision.reason;
      }
      egress_condition.notify_one();
    }
    return false;
  }

  void on_joint_state(sensor_msgs::msg::JointState::ConstSharedPtr message,
                      const rclcpp::MessageInfo& info) {
    // Freeze first callback receipt before discovery/graph/FK work.
    const int64_t receipt_ros = node.get_clock()->now().nanoseconds();
    const int64_t receipt_steady = steady_ns();
    JointIngressTrace trace;
    trace.available = true;
    trace.callback_entry = {receipt_steady, diagnostic_thread_cpu_ns()};
    const auto& metadata = info.get_rmw_message_info();
    trace.rmw_source_timestamp_ns = metadata.source_timestamp;
    trace.rmw_received_timestamp_ns = metadata.received_timestamp;
    ++received;
    const auto current = owner();
    trace.owner_end = ingress_clock_point();
    if (!discovery_ready(current, receipt_ros, receipt_steady)) return;
    if (current.status != platform::LiveJointStateOwnerStatus::kMatched) {
      reject({false, false, "live_joint_source_owner_invalid", 0, {}, {}},
             receipt_ros, receipt_steady);
      return;
    }
    if (message->header.stamp.nanosec >= 1000000000U) {
      reject({false, true, "live_joint_source_stamp_invalid", 0, {}, {}},
             receipt_ros, receipt_steady);
      return;
    }
    const int64_t source = static_cast<int64_t>(message->header.stamp.sec) * 1000000000LL +
                           message->header.stamp.nanosec;
    const auto decision = gate->evaluate({source, receipt_ros, message->name, message->position,
        true, gid_hex(info.get_rmw_message_info().publisher_gid) == current.endpoint_gid});
    // This interval includes discovery/header/GID checks and the original gate;
    // it is not claimed to measure the gate's internal implementation alone.
    trace.gate_end = ingress_clock_point();
    if (!decision.accepted) {
      reject(decision, receipt_ros, receipt_steady);
      return;
    }
    const auto result = kinematics->ingest(decision, receipt_ros, receipt_steady, trace);
    if (!result.accepted) {
      reject({false, result.fault_latched, result.reason, 0, {}, {}},
             receipt_ros, receipt_steady);
      return;
    }
    const auto sequence = ++accepted;
    {
      std::lock_guard<std::mutex> lock(identity_mutex);
      accepted_identity = {source_owner_fqn,
        current.endpoint_gid, sequence, result.evidence};
    }
    Proof proof;
    proof.message.header = message->header;
    proof.message.name = decision.names;
    proof.message.position = decision.positions;
    proof.source_stamp_ns = decision.source_stamp_ns;
    proof.receipt_steady_ns = receipt_steady;
    {
      std::lock_guard<std::mutex> lock(egress_mutex);
      if (pending_proof) ++proof_overwritten;
      pending_proof = std::move(proof);
    }
    egress_condition.notify_one();
  }

  void egress() {
    while (true) {
      std::optional<Proof> proof;
      std::optional<std::string> reason;
      {
        std::unique_lock<std::mutex> lock(egress_mutex);
        egress_condition.wait(lock, [this]() {return stopping || pending_proof || pending_reason;});
        if (stopping) return;
        proof = std::move(pending_proof);
        reason = std::move(pending_reason);
        pending_proof.reset();
        pending_reason.reset();
      }
      try {
        if (reason) {
          std_msgs::msg::String message;
          message.data = *reason;
          reason_publisher->publish(message);
        }
        if (proof) {
          const int64_t now_ros = node.get_clock()->now().nanoseconds();
          const int64_t now_steady = steady_ns();
          const int64_t age = now_ros - proof->source_stamp_ns;
          if (age < -future_tolerance_ns || age > maximum_age_ns ||
              now_steady < proof->receipt_steady_ns ||
              now_steady-proof->receipt_steady_ns > maximum_age_ns ||
              kinematics->lookup(proof->source_stamp_ns, [this] {
                return JointKinematics::ClockSample{node.get_clock()->now().nanoseconds(), steady_ns()};
              }).status !=
                  JointKinematicsStatus::Ready) {
            ++proof_expired;
          } else {
            publisher->publish(proof->message);
            ++proof_published;
          }
        }
      } catch (const std::exception& error) {
        // A broken publisher cannot kill the process while leaving apparently
        // valid in-process joint evidence. Revoke it and latch closed.
        const int64_t now_ros = node.get_clock()->now().nanoseconds();
        (void)kinematics->ingest({false, true, "joint_ingress_egress_failed", 0, {}, {}},
                                 now_ros, steady_ns());
        RCLCPP_ERROR(node.get_logger(), "joint ingress egress failed: %s", error.what());
        return;
      }
    }
  }

  JointIngress& node;
  std::shared_ptr<JointKinematics> kinematics;
  JointIngressContract receipt;
  std::string source_topic, owner_name, owner_namespace, source_owner_fqn;
  int64_t maximum_age_ns{0}, future_tolerance_ns{0}, discovery_started_ns{0};
  bool discovery_timeout_reported{false};
  bool fault_reported{false};
  std::unique_ptr<platform::LiveJointStateBridgeGate> gate;
  std::unique_ptr<platform::LiveJointStateDiscoveryGate> discovery_gate;
  rclcpp::TimerBase::SharedPtr discovery_timer;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_publisher;
  std::mutex egress_mutex;
  mutable std::mutex identity_mutex;
  JointAcceptedIdentity accepted_identity;
  std::condition_variable egress_condition;
  std::optional<Proof> pending_proof;
  std::optional<std::string> pending_reason;
  bool stopping{false};
  std::thread egress_thread;
  std::atomic<uint64_t> received{0}, accepted{0}, proof_overwritten{0}, proof_published{0},
                         proof_expired{0};
};

JointIngress::JointIngress(std::shared_ptr<JointKinematics> kinematics,
                           const rclcpp::NodeOptions& options)
    : Node("sunhorizon_live_joint_state_bridge", options),
      impl_(std::make_unique<Impl>(*this, std::move(kinematics))) {}
JointIngress::~JointIngress() = default;
JointIngressContract JointIngress::contract() const {return impl_->receipt;}
JointIngressCounters JointIngress::counters() const {
  return {impl_->received.load(), impl_->accepted.load(), impl_->proof_overwritten.load(),
          impl_->proof_published.load(), impl_->proof_expired.load()};
}
JointAcceptedIdentity JointIngress::accepted_identity() const {
  std::lock_guard<std::mutex> lock(impl_->identity_mutex);
  return impl_->accepted_identity;
}
std::shared_ptr<JointKinematics> JointIngress::kinematics() const {return impl_->kinematics;}
std::shared_ptr<JointIngress> make_joint_ingress(
    std::shared_ptr<JointKinematics> kinematics, const rclcpp::NodeOptions& options) {
  return std::make_shared<JointIngress>(std::move(kinematics), options);
}
}  // namespace sunhorizon_perception_runtime
