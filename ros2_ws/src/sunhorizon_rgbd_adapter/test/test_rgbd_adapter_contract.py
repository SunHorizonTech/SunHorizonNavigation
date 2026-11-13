from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src/rgbd_adapter_node.cpp"
RGBD_STATUS = PACKAGE.parent / "sunhorizon_navigation_interfaces/msg/RgbdStatus.msg"


def function_body(source: str, name: str, next_name: str) -> str:
    start = source.index(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_successful_pair_clears_only_the_transient_rejection_reason():
    source = SOURCE.read_text(encoding="utf-8")
    process_pair = function_body(
        source, "void process_pair", "void on_belly_depth"
    )

    snapshot = (PACKAGE / "include/sunhorizon_rgbd_adapter/rgbd_status_snapshot.hpp").read_text()
    assert "state_.last_rejection_reason.clear();" in snapshot
    assert "status_.accepted(" in process_pair
    assert "rejection_stamps_.clear()" not in snapshot
    assert "accepted_pairs = 0" not in process_pair



def test_current_failure_remains_visible_until_a_successful_pair():
    source = SOURCE.read_text(encoding="utf-8")
    process_pair = function_body(
        source, "void process_pair", "void on_belly_depth"
    )
    publish_status = function_body(
        source, "void publish_status", "RgbdPairValidator validator_"
    )

    assert process_pair.count("state.last_rejection_reason =") == 3
    assert "last_pair_rejection=" in publish_status


def test_all_raw_sensor_subscriptions_are_latest_only():
    source = SOURCE.read_text(encoding="utf-8")
    subscriptions = function_body(
        source, "color_sub_ = create_subscription", "const auto period"
    )

    assert source.count(
        "rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile()"
    ) == 1
    assert subscriptions.count("latest_sensor_qos()") == 6
    assert "rclcpp::SensorDataQoS()" not in subscriptions


def test_output_publishers_keep_sensor_data_qos():
    source = SOURCE.read_text(encoding="utf-8")
    publishers = function_body(
        source, "color_pub_ = create_publisher", "color_sub_ = create_subscription"
    )

    assert publishers.count("rclcpp::SensorDataQoS()") == 4


def test_status_preserves_signed_measurement_age_and_reports_cloud_age():
    source = SOURCE.read_text(encoding="utf-8")
    publish_status = function_body(
        source, "void publish_status", "RgbdPairValidator validator_"
    )

    freshness = (
        PACKAGE / "src/rgbd_freshness.cpp"
    ).read_text(encoding="utf-8")
    assert "return now_sec - measurement_stamp_sec;" in freshness
    assert "age_sec >= 0.0" in freshness
    assert "std::max(0.0, now -" not in publish_status
    assert "output.chest_pointcloud_age_sec" in publish_status
    assert "output.belly_pointcloud_age_sec" in publish_status
    assert publish_status.count("measurement_fresh(") == 4


def test_queue_size_is_signed_bounded_and_robot_a_keeps_latest_pair_only():
    source = SOURCE.read_text(encoding="utf-8")
    config = (
        PACKAGE / "config/rgbd_adapter_robot_a.prototype-accepted.yaml"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter<int64_t>("queue_size", 2)' in source
    assert "requested_queue_size < 2 || requested_queue_size > 64" in source
    assert "queue_size: 2" in config


def test_all_safety_numeric_parameters_must_be_finite():
    source = SOURCE.read_text(encoding="utf-8")
    validate = function_body(source, "void validate_parameters", "bool depth_image_valid")

    assert "std::isfinite(freshness_timeout_sec_)" in validate
    assert "std::isfinite(max_pair_delta_sec_)" in validate
    assert "std::isfinite(depth_scale_m_)" in validate


def test_cloud_age_fields_are_appended_after_the_legacy_message_prefix():
    message = RGBD_STATUS.read_text(encoding="utf-8")

    reason = message.index("string reason")
    assert reason < message.index("float64 chest_pointcloud_age_sec")
    assert reason < message.index("float64 belly_pointcloud_age_sec")


def test_component_filter_is_opt_in_and_changes_only_chest_cloud():
    source = SOURCE.read_text(encoding="utf-8")
    assert 'declare_parameter("chest_component_filter_enabled", false)' in source
    process = function_body(source, "void process_pair", "void on_belly_depth")
    assert process.index('depth_pub_->publish(normalized_depth)') < process.index('depth_component_mask(')
    assert process.index('color_pub_->publish(color)') < process.index('depth_component_mask(')
    belly = function_body(source, "void on_belly_depth", "void publish_status")
    assert 'depth_component_mask' not in belly
    assert 'make_pointcloud(*input, *belly_info_)' in belly
