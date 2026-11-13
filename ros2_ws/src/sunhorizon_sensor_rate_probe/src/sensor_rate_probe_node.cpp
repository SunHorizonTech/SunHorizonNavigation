#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sunhorizon_sensor_rate_probe/rate_tracker.hpp"

namespace
{

double steady_seconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

template<typename MessageT>
double header_seconds(const MessageT & message)
{
  return static_cast<double>(message.header.stamp.sec) +
         static_cast<double>(message.header.stamp.nanosec) * 1.0e-9;
}

void print_snapshot(
  const char * name,
  const sunhorizon_sensor_rate_probe::RateSnapshot & snapshot,
  bool trailing_comma)
{
  std::cout << "  \"" << name << "\": {\n"
            << "    \"samples\": " << snapshot.samples << ",\n"
            << "    \"receive_interval_count\": " << snapshot.receive_interval_count << ",\n"
            << "    \"receive_span_sec\": " << snapshot.receive_span_sec << ",\n"
            << "    \"receive_rate_hz\": " << snapshot.receive_rate_hz << ",\n"
            << "    \"header_samples\": " << snapshot.header_samples << ",\n"
            << "    \"header_interval_count\": " << snapshot.header_interval_count << ",\n"
            << "    \"header_span_sec\": " << snapshot.header_span_sec << ",\n"
            << "    \"header_rate_hz\": " << snapshot.header_rate_hz << ",\n"
            << "    \"header_regressions\": " << snapshot.header_regressions << ",\n"
            << "    \"header_duplicates\": " << snapshot.header_duplicates << "\n"
            << "  }" << (trailing_comma ? "," : "") << "\n";
}

}  // namespace

class SensorRateProbe final : public rclcpp::Node
{
public:
  SensorRateProbe()
  : Node("sunhorizon_sensor_rate_probe"), started_sec_(steady_seconds())
  {
    points_topic_ =
      declare_parameter<std::string>("points_topic", "/sunhorizon/raw/airy/points");
    imu_topic_ =
      declare_parameter<std::string>("imu_topic", "/sunhorizon/raw/airy/imu");
    duration_sec_ = declare_parameter<double>("duration_sec", 15.0);
    minimum_cloud_rate_hz_ = declare_parameter<double>("minimum_cloud_rate_hz", 9.0);
    minimum_imu_rate_hz_ = declare_parameter<double>("minimum_imu_rate_hz", 190.0);
    if (duration_sec_ <= 0.0) {
      throw std::invalid_argument("duration_sec must be positive");
    }

    auto qos = rclcpp::SensorDataQoS().keep_last(5);
    points_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      points_topic_, qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        points_rate_.observe(steady_seconds(), header_seconds(*message));
      });
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        imu_rate_.observe(steady_seconds(), header_seconds(*message));
      });
  }

  [[nodiscard]] bool done() const
  {
    return steady_seconds() - started_sec_ >= duration_sec_;
  }

  [[nodiscard]] bool report() const
  {
    const auto points = points_rate_.snapshot();
    const auto imu = imu_rate_.snapshot();
    const auto point_publishers = count_publishers(points_topic_);
    const auto imu_publishers = count_publishers(imu_topic_);
    const bool pass =
      point_publishers == 1 && imu_publishers == 1 &&
      points.receive_rate_hz >= minimum_cloud_rate_hz_ &&
      imu.receive_rate_hz >= minimum_imu_rate_hz_ &&
      points.header_regressions == 0 && imu.header_regressions == 0 &&
      points.header_duplicates == 0 && imu.header_duplicates == 0;

    std::cout << std::setprecision(15)
              << "{\n"
              << "  \"status\": \"" << (pass ? "PASS" : "FAIL") << "\",\n"
              << "  \"duration_sec\": " << steady_seconds() - started_sec_ << ",\n"
              << "  \"points_topic\": \"" << points_topic_ << "\",\n"
              << "  \"imu_topic\": \"" << imu_topic_ << "\",\n"
              << "  \"point_publishers\": " << point_publishers << ",\n"
              << "  \"imu_publishers\": " << imu_publishers << ",\n"
              << "  \"minimum_cloud_rate_hz\": " << minimum_cloud_rate_hz_ << ",\n"
              << "  \"minimum_imu_rate_hz\": " << minimum_imu_rate_hz_ << ",\n";
    print_snapshot("points", points, true);
    print_snapshot("imu", imu, false);
    std::cout << "}\n";
    return pass;
  }

private:
  double started_sec_;
  double duration_sec_{15.0};
  double minimum_cloud_rate_hz_{9.0};
  double minimum_imu_rate_hz_{190.0};
  std::string points_topic_;
  std::string imu_topic_;
  sunhorizon_sensor_rate_probe::RateTracker points_rate_;
  sunhorizon_sensor_rate_probe::RateTracker imu_rate_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorRateProbe>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !node->done()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool pass = node->report();
  executor.remove_node(node);
  rclcpp::shutdown();
  return pass ? 0 : 2;
}
