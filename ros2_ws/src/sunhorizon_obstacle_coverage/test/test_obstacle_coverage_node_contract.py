from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "src/obstacle_coverage_node.cpp"
FUSION_CONFIG = PACKAGE / "config/obstacle_coverage_fusion_shadow.yaml"


def test_large_sensor_subscriptions_are_latest_only_and_processed_belly_is_opt_in():
    source = SOURCE.read_text(encoding="utf-8")

    assert "rclcpp::KeepLast(1)" in source
    assert source.count("latest_sensor_qos()") == 4
    assert source.count("latest_status_qos()") == 3  # Opt-in spatial input, plus RGB-D.
    assert "rclcpp::SensorDataQoS()" not in source
    assert "rclcpp::QoS(10).reliable()" not in source
    assert '"chest_points_topic"' not in source
    assert '"belly_points_topic"' not in source
    assert '"belly_obstacle_points_topic", "", descriptor' in source
    assert 'if (require_processed_belly_)' in source


def test_rgbd_status_carries_source_time_and_pointcloud_health_without_full_cloud_copy():
    source = SOURCE.read_text(encoding="utf-8")

    assert "input->chest_pair_age_sec," in source
    assert "input->chest_pointcloud_age_sec" in source
    assert "input->belly_depth_age_sec," in source
    assert "input->belly_pointcloud_age_sec" in source
    assert source.count("oldest_source_stamp(") == 1
    assert "belly_health_policy_.source_stamp(" in source
    assert "input->chest_pointcloud_healthy" in source
    assert "input->belly_pointcloud_healthy" in source
    assert "transform_available(input_frame_id)" in source


def test_processed_age_deduplication_is_only_direct_spatial_and_keeps_final_gates():
    source = SOURCE.read_text(encoding="utf-8")
    assert "BellyHealthPolicy(bool(direct_),spatial_required)" in source
    assert source.count("if (belly_health_policy_.require_processed(require_processed_belly_))") == 2
    assert "spatial_tracker_ && (!spatial.valid || spatial.disposition != 1)" in source
    assert "direct_coverage_revoked:" in source
    assert "envelope.spatial = spatial_wire" in source


def test_rgbd_global_state_does_not_cross_poison_sources():
    source = SOURCE.read_text(encoding="utf-8")

    assert "evaluate_rgbd_coverage_health" in source
    assert "input->state == RgbdStatus::STATE_HEALTHY" not in source


def test_timing_diagnostics_do_not_relax_fail_closed_freshness():
    source = SOURCE.read_text(encoding="utf-8")

    assert "receipt_stamp_sec" in source
    assert "header_age=" in source
    assert "receipt_age=" in source
    assert "queue_delay=" in source
    assert "RCLCPP_ERROR_THROTTLE" in source
    assert "1000," in source


def test_large_cloud_status_and_fail_closed_timer_are_independently_scheduled():
    source = SOURCE.read_text(encoding="utf-8")

    for callback_group in (
        "airy_callback_group_",
        "rgbd_callback_group_",
        "status_callback_group_",
    ):
        assert callback_group in source
    assert source.count("rclcpp::CallbackGroupType::MutuallyExclusive") == 6
    assert "airy_options.callback_group = airy_callback_group_" in source
    assert "rgbd_options.callback_group = rgbd_callback_group_" in source
    assert "status_callback_group_);" in source
    main = (PACKAGE / "src/obstacle_coverage_main.cpp").read_text()
    assert "rclcpp::executors::MultiThreadedExecutor executor(" in main
    assert "rclcpp::ExecutorOptions(),3U" in main


def test_observation_lock_is_not_held_during_tf_or_publish_work():
    source = SOURCE.read_text(encoding="utf-8")

    assert "std::mutex observations_mutex_" in source
    inputs_start = source.index("    CoverageInputs inputs;")
    snapshot_start = source.rindex("    {", 0, inputs_start)
    snapshot_end = source.index("\n    }\n\n", snapshot_start) + len("\n    }\n\n")
    snapshot = source[snapshot_start:snapshot_end]
    assert "std::lock_guard<std::mutex> lock(observations_mutex_)" in snapshot
    assert "airy_cloud = airy_cloud_;" in snapshot
    assert "chest_status = chest_status_;" in snapshot
    assert "belly_status = belly_status_;" in snapshot
    assert "cloud_source(" not in snapshot
    assert "rgbd_source(" not in snapshot
    assert "status_pub_->publish" not in snapshot
    now_index = source.index("const auto now = get_clock()->now();", snapshot_end)
    transform_work_index = source.index("inputs.belly = rgbd_source(", inputs_start)
    evaluate_index = source.index("evaluator_->evaluate(inputs)", now_index)
    assert snapshot_end <= inputs_start < transform_work_index < now_index < evaluate_index


def test_robot_a_integrated_fusion_freshness_matches_measured_source_age():
    config = FUSION_CONFIG.read_text(encoding="utf-8")

    assert "measured 0.918 s belly and 1.139 s" in config
    assert "airy_freshness_sec: 0.65" in config
    assert "chest_freshness_sec: 1.20" in config
    assert "belly_freshness_sec: 1.00" in config


def test_typed_calculation_uses_one_post_tf_locked_clock_without_publish_or_tf_work():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("    auto [now, decision, region] = [&]() {")
    end = source.index("      // Preserve the legacy sampling order", start)
    typed = source[start:end]
    assert typed.count("get_clock()->now()") == 1
    assert typed.count("steady_nanoseconds()") == 1
    assert typed.index("lock(observations_mutex_)") < typed.index("get_clock()->now()")
    assert "inputs.now_sec = now.seconds();" in typed
    assert "evaluator_->evaluate(inputs)" in typed
    assert "region_tracker_->evaluate(now.nanoseconds(), steady)" in typed
    assert "return std::make_tuple(now, decision, region);" in typed
    assert "publish(" not in typed and "transform_available(" not in typed
    assert source.index("cloud_source(belly_cloud)") < start
    assert source.index("output.header.stamp = now;", end) > end


def test_direct_inputs_reuse_original_owner_without_subscriptions_or_fake_message_info():
    source=SOURCE.read_text()
    for lane in ('airy','belly','rgbd','region','spatial'):
        assert 'if(!direct_)'+lane+'_sub_ = create_subscription' in source
    direct=source.split('SpatialDeliveryResult receive_spatial(',1)[1].split('  void revoke(',1)[0]
    assert 'direct_endpoint()' in direct and 'accept_spatial' in direct
    assert 'MessageInfo' not in direct
    assert 'spatial_publisher->get_gid()' in source
    assert 'if(direct_revoked_.exchange(true))return;' in source
    assert 'transforms_->lookupTransform' in source


def test_standalone_tf_uses_existing_executor_without_hidden_spin_thread():
    source=SOURCE.read_text()
    assert 'tf_buffer_,this,false,' in source
    assert 'dynamic_options.callback_group=tf_callback_group_' in source
    assert 'static_options.callback_group=tf_callback_group_' in source
    assert 'std::make_unique<tf2_ros::TransformListener>(tf_buffer_)' not in source
    main=(PACKAGE/'src/obstacle_coverage_main.cpp').read_text()
    assert 'MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),3U)' in main
    assert 'executor.add_node(node);executor.spin()' in main
