from pathlib import Path
import unittest
ROOT = Path(__file__).resolve().parents[1]
SOURCE = (ROOT / 'src/navigation_runtime.cpp').read_text()
HEADER = (ROOT / 'include/sunhorizon_navigation_runtime/navigation_runtime.hpp').read_text()
MAIN = (ROOT / 'src/main.cpp').read_text()


class ActualRuntimeWiring(unittest.TestCase):
    def test_rgbd_routing_does_not_create_airy_obstacle_workers_or_consumers(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        build=assembly.split('      if(obstacle_uses_airy(options.obstacle_source_mode)) {',1)[1]
        for operation in ('bindDirectCollisionResultSource(collision,airy.collision_source',
                'airy_ground = std::make_unique','airy_compute = std::make_unique'):
            self.assertIn(operation,build.split('      canonical = ',1)[0])
        consume=assembly.split('bool Nav2Assembly::consume_airy(',1)[1]
        self.assertLess(consume.index('!obstacle_uses_airy(impl_->options.obstacle_source_mode)'),
                        consume.index('airy_compute->replace'))
        self.assertIn('if(obstacle_uses_airy(options.obstacle_source_mode))coverage_airy=',SOURCE)
        self.assertIn('if(sinks.airy_frame)sinks.airy_frame(message,stamp)',SOURCE)
        self.assertIn('require_obstacle_coverage_profile(options.obstacle_source_mode,',SOURCE)

    def test_localization_outage_preserves_owners_and_latches_actual_control(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        hold=assembly.split('  void hold_localization_locked(',1)[1].split('  void forward_odometry_locked()',1)[0]
        self.assertIn('canonical_motion_hold.store(true)',hold)
        self.assertNotIn('fault(',hold)
        self.assertNotIn('canonical_motion_hold.store(false)',assembly)
        forward=assembly.split('  void forward_odometry_locked()',1)[1].split('  void receive(const State',1)[0]
        self.assertLess(forward.index('canonical_motion_hold.load()'),forward.index('submitDirectOdometry'))
        self.assertIn('hold_localization_locked(failure)',forward)
        state=assembly.split('  void receive(const State',1)[1].split('  bool transforms_ready()',1)[0]
        self.assertIn('latest_state = state; state_sequence = state->sequence',state)
        self.assertNotIn('throw std::runtime_error("canonical_localization_',state)
        activation=assembly.split('  void activation_loop()',1)[1].split('  void freeze_collision_height_configuration()',1)[0]
        self.assertIn('controller->deactivate().id() != LifecycleState::PRIMARY_STATE_INACTIVE',activation)
        self.assertIn('controller_suspended.store(true)',activation)
        self.assertIn('canonical_manager_owner_failed',activation)
        self.assertNotIn('canonical_localization_watchdog_expired',activation)
        self.assertIn('if (!healthy_once && startup.should_poll(assembly->motion_held(),startup_poll_now))',MAIN)
        self.assertIn('spatial_startup_original_deadline_expired_while_motion_held',MAIN)
        self.assertIn('if (peripherals_complete && !assembly->initialized())',MAIN)
        self.assertIn('!assembly->motion_held() && std::chrono::steady_clock::now() >= deadline',MAIN)
        self.assertIn('bool Nav2Assembly::active() const {return initialized() && !motion_held();}',assembly)
        self.assertIn('canonical_localization_internal_fault:',assembly)


    def test_held_before_first_spatial_publication_does_not_fault_perception(self):
        source=(ROOT.parent/'sunhorizon_perception_runtime/src/runtime.cpp').read_text()
        direct=source.split('  void direct_evidence(',1)[1].split('  void observe_evidence(',1)[0]
        observer=source.split('  void publish_evidence(',1)[1].split('  void tick()',1)[0]
        self.assertLess(direct.index('publication_allowed(dm::steady_ns())'),direct.index('spatial_sink(immutable)'))
        self.assertLess(observer.index('publication_allowed(dm::steady_ns())'),observer.index('evidence_pub->publish(e)'))
        self.assertNotIn('fault("spatial_startup_deadline',source)
        self.assertNotIn('fault("spatial_first_publication_original_deadline',source)
        self.assertIn('!assembly->motion_held() && std::chrono::steady_clock::now() >= deadline',MAIN)

    def test_airy_invalid_returns_never_refresh_consumer_evidence(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        receive=assembly.split('  void receive_airy_filtered(',1)[1].split('  std::string health_failure_locked()',1)[0]
        self.assertLess(receive.index('detail::make_classified_airy_packet(delivery.cloud,ground,output->ground_blocked,output->sample_ids)'),
                        receive.index('if(!classified.deliverable)return;'))
        self.assertLess(receive.index('if(!classified.deliverable)return;'),receive.index('airy_local->admit'))
        self.assertIn('delivery.cloud=std::move(classified.frame)',receive)
        for key in ('source_epoch','sequence','source_ns','input_points','finite_points','invalid_points','delivered','ground_source_unavailable'):
            self.assertIn('"'+key+'"',assembly.split('  void publish_timings()',1)[1].split('  void bind_costmap_timing',1)[0])

    def test_airy_self_and_ground_have_distinct_latest_stages_with_original_owners(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        handoff=assembly.split('  void receive_airy_filtered(',1)[1].split('  void process_airy_ground(',1)[0]
        ground=assembly.split('  void process_airy_ground(',1)[1].split('  std::string health_failure_locked()',1)[0]
        self.assertEqual(assembly.count('options.filtered_airy(output)'),1)
        self.assertLess(handoff.index('options.filtered_airy(output)'),handoff.index('airy_ground->replace'))
        self.assertIn('AiryGroundInput{output,nav2_util::directSteadyNowNs()}',handoff)
        self.assertIn('retired.get()==offered || stats.closed || stats.errors',handoff)
        self.assertNotIn('make_classified_airy_packet',handoff)
        self.assertNotIn('airy_local->admit',handoff)
        self.assertNotIn('observe_admission',handoff)
        self.assertIn('LatestFrameWorker<AiryGroundInput>',assembly)
        self.assertIn('struct AiryGroundInput {AiryOutput output; int64_t queued_ns{0};}',assembly)
        self.assertLess(assembly.index('airy_ground = std::make_unique'),assembly.index('airy_compute = std::make_unique'))
        self.assertIn('if (!running() || !activated.load()) return;',ground)
        before_fanout=ground.split('    const bool local = airy_local->admit',1)[0]
        self.assertTrue(before_fanout.rstrip().endswith('if (!running() || !activated.load()) return;'))
        self.assertIn('delivery.cloud.ray_origins=output->ray_origins',ground)
        self.assertIn('detail::make_classified_airy_packet(delivery.cloud,ground,output->ground_blocked,output->sample_ids)',ground)
        for original in ('stamp.source_ns','stamp.first_receipt_ros_ns','stamp.first_receipt_steady_ns',
                         'stamp.raw_source_ns','stamp.sequence','stamp.epoch'):
            self.assertIn(original,ground)
        self.assertEqual(assembly.count('airy_filter->receive_result(input)'),1)
        stages=assembly.split('      airy_ground = std::make_unique',1)[1].split('      canonical = ',1)[0]
        separate=stages.split('      airy_compute = ',1)
        self.assertIn('"airy.ground_fanout_returned",input.queued_ns,begin,begin,end',separate[0])
        self.assertIn('process_airy_ground(input.output)',separate[0])
        self.assertIn('"airy.rig_filter_returned",input->queue_admitted_steady_ns,begin,begin,end',separate[1])
        self.assertNotIn('process_airy_ground',separate[1])
        self.assertNotIn('make_classified_airy_packet',separate[1])

    def test_airy_both_latest_stages_stop_and_join_before_filter_and_plugins(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        stop=assembly.split('  void stop() {',1)[1].split('Nav2Assembly::Nav2Assembly',1)[0]
        for stage in ('airy_compute','airy_ground'):
            self.assertLess(stop.index('stopping.exchange(true)'),stop.index(stage+'->request_stop()'))
            self.assertLess(stop.index(stage+'->request_stop()'),stop.index('airy_compute->join()'))
            self.assertLess(stop.index(stage+'->join()'),stop.index('airy_filter->stop()'))
            self.assertLess(stop.index(stage+'->join()'),stop.index('node->cleanup()'))
        self.assertLess(stop.index('airy_compute->join()'),stop.index('airy_ground->join()'))

    def test_height_clipped_query_is_rejected_and_latched_before_any_production_acquisition(self):
        start=SOURCE.split('  void start(){',1)[1].split('  void stop(){',1)[0]
        refusal=start.index('if(options.spatial_query.height_limited())')
        latch=start.index('fault("low_obstacle_safety_unvalidated_offline_only")')
        rejection=start.index('throw std::invalid_argument("low_obstacle_safety_unvalidated_offline_only")')
        self.assertLess(refusal,latch)
        self.assertLess(latch,rejection)
        for effect in ('if(!sinks.perception_output)', 'executor=std::make_unique',
                       'camera::create_camera_runtime(', 'inputs_ready.store(true)', 'airy_owner->start()'):
            self.assertLess(rejection,start.index(effect))
        self.assertNotIn('collision_height_bound',start)

    def test_collision_height_proof_follows_actual_configuration_before_acquisition(self):
        assembly=(ROOT/'src/nav2_assembly.cpp').read_text()
        start=assembly.split('  void start() {',1)[1].split('  void stop()',1)[0]
        self.assertLess(start.index('freeze_collision_height_configuration();'),start.index('planner->configure()'))
        self.assertLess(start.index('controller->configure()'),start.index('bind_collision_height_responsibility();'))
        activation=assembly.split('  void activation_loop()',1)[1].split('  void freeze_collision_height_configuration()',1)[0]
        self.assertLess(activation.index('bind_collision_height_responsibility();'),activation.index('activated.store(true)'))
        self.assertLess(MAIN.index('assembly->start();'),MAIN.index('navigation->bind_collision_height_responsibility(*binding)'))
        self.assertLess(MAIN.index('navigation->bind_collision_height_responsibility(*binding)'),MAIN.index('navigation->start();'))
        canonical=SOURCE.split('  void receive_canonical(std::shared_ptr<const manager::CanonicalLocalizationState> state)',1)[1].split('  void receive_canonical(',1)[0]
        self.assertLess(canonical.index('planar(state->map_to_odom.transform)'),canonical.index('transforms->setTransform'))
        self.assertLess(canonical.index('p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w'),canonical.index('transforms->setTransform'))

    def test_perception_first_fault_snapshot_preserves_latch_without_nested_owner_locks(self):
        root=ROOT.parent/'sunhorizon_perception_runtime'
        source=(root/'src/runtime.cpp').read_text()
        header=(root/'include/sunhorizon_perception_runtime/runtime.hpp').read_text()
        self.assertIn('std::string fault_reason() const;',header)
        fault=source.split('  bool fault(const std::string& reason){',1)[1].split('  void stop(){',1)[0]
        self.assertLess(fault.index('lock(state_mutex)'),fault.index('if(terminal.load())return false;'))
        self.assertLess(fault.index('if(terminal.load())return false;'),fault.index('fault_reason=reason.substr(0,512)'))
        self.assertLess(fault.index('fault_reason=reason.substr(0,512)'),fault.index('terminal.store(true)'))
        snapshot=source.split('std::string PerceptionRuntime::fault_reason()const{',1)[1].split('\n}',1)[0]
        self.assertLess(snapshot.index('lock(impl_->state_mutex)'),snapshot.index('return impl_->fault_reason'))
        self.assertNotIn('stopping',snapshot)
        stop=source.split('  void stop(){',1)[1].split('  JointKinematicsLookup wait_fk(',1)[0]
        self.assertNotIn('fault_reason=',stop)
        propagated=SOURCE.split('std::string NavigationRuntime::failure_reason()const{',1)[1].split('\n}',1)[0]
        self.assertIn('if(!impl_->reason.empty())return impl_->reason;}',propagated)
        self.assertLess(propagated.index('return impl_->reason;}'),propagated.index('perception_owner->fault_reason()'))
        self.assertIn('"perception_owner_failed:"+reason',propagated)

    def test_direct_identity_observer_preserves_original_clocks_without_sdk_publish(self):
        self.assertIn('direct_owner_snapshot() const',HEADER)
        snapshot=SOURCE.split('  diagnostic_msgs::msg::DiagnosticArray identity_snapshot()',1)[1].split('  void admit_manager(',1)[0]
        self.assertIn('joints->accepted_identity()',snapshot)
        for key in ('sample.receipt_ros_ns','sample.receipt_steady_ns','joint.fk.receipt_stamp_ns','joint.source_publisher_gid'):
            self.assertIn(key,snapshot)
        self.assertGreater(snapshot.index('result.header.stamp=control->now()'),snapshot.index('camera_evidence.snapshot()'))
        callback=SOURCE.split('[this](auto frame){',1)[1].split('perception_owner->admit(frame)',1)[0]
        self.assertIn('frame->source_identity',callback)
        for forbidden in ('publish(', 'get_parameter(', 'ifstream', 'process_startticks('):
            self.assertNotIn(forbidden,callback)
        self.assertIn('create_wall_timer(std::chrono::seconds(1)',SOURCE)
        self.assertIn('camera::LatestFrameWorker<diagnostic_msgs::msg::DiagnosticArray>',SOURCE)
        joint=(ROOT.parent/'sunhorizon_perception_runtime/src/joint_ingress.cpp').read_text()
        accepted=joint.split('  void on_joint_state(',1)[1].split('  void egress()',1)[0]
        self.assertLess(accepted.index('if (!decision.accepted)'),accepted.index('accepted_identity ='))
        self.assertLess(accepted.index('if (!result.accepted)'),accepted.index('accepted_identity ='))
        self.assertIn('current.endpoint_gid, sequence, result.evidence',accepted)

    def test_livo_timing_observer_keeps_source_and_output_receipts_distinct(self):
        snapshot=SOURCE.split('  diagnostic_msgs::msg::DiagnosticArray identity_snapshot()',1)[1].split('  void admit_manager(',1)[0]
        for field in ('original_imu','latest_dispatched_imu','anchor_generation',
                      'vendor_admission_steady_ns','vendor_dispatch_steady_ns',
                      'propagation_wall_ns','propagation_cpu_ns',
                      'epoch_to_corrected_wall_ns','epoch_to_corrected_cpu_ns'):
            self.assertIn('value.'+field,snapshot)
        for role in ('corrected','propagated'):
            callback=SOURCE.split('  void receive_vendor(vendor::'+role.title()+'State::ConstSharedPtr value,',1)[1].split('\n  }',1)[0]
            self.assertIn('value->source_ns,value->receipt_ros_ns',callback)
            self.assertLess(callback.index('adapter->receive_'+role+'_odometry'),callback.index('lock(vendor_timing_mutex)'))
            self.assertIn('vendor_timing.'+role+'.swap(value)',callback)
            self.assertNotIn('identity_field',callback)
            self.assertNotIn('publish(',callback)

    def test_one_actual_owner_and_shared_camera(self):
        self.assertEqual(SOURCE.count('camera::create_camera_runtime('), 1)
        self.assertEqual(SOURCE.count('airy::create_direct_driver('), 1)
        self.assertEqual(SOURCE.count('vendor::DirectRuntime::create_with_fault_info('), 1)
        self.assertEqual(SOURCE.count('ndt::make_ndt_direct_runtime('), 1)
        self.assertEqual(SOURCE.count('normalization::make_airy_direct_adapter('), 1)
        self.assertIn('perception_owner->admit(frame)', SOURCE)
        self.assertIn('image_lane->admit(std::move(frame))', SOURCE)
        self.assertNotIn('create_subscription', SOURCE)
        self.assertNotIn('cmd_vel', SOURCE)

    def test_actual_vendor_image_mode_owns_adapter_requirement_and_rgb_lane(self):
        creation=SOURCE.index('vendor_owner=vendor::DirectRuntime::create_with_fault_info(')
        mode=SOURCE.index('vendor_owner->node()->get_parameter("common.img_en").as_int()!=0')
        binding=SOURCE.index('adapter_sinks.vendor_images_enabled=livo_visual_inputs_required')
        adapter=SOURCE.index('adapter=livo::make_livo_direct_adapter(')
        self.assertLess(creation,mode)
        self.assertLess(mode,binding)
        self.assertLess(binding,adapter)
        self.assertIn('if(livo_visual_inputs_required)image_lane=std::make_unique<detail::BoundedLane<Camera>>(2,',SOURCE)
        callback=SOURCE.split('[this](auto frame){',1)[1].split('[this](const auto & error)',1)[0]
        self.assertLess(callback.index('camera_evidence.accepted'),callback.index('perception_owner->admit(frame)'))
        self.assertLess(callback.index('perception_owner->admit(frame)'),callback.index('frame->role=="chest"&&image_lane&&'))
        self.assertIn('!image_lane->admit(std::move(frame)))fault("camera_rgb_lane_overflow")',callback)
        self.assertNotIn('get_parameter',callback)
        self.assertNotIn('camera_owner->stop',callback)
        self.assertNotIn('require_image',SOURCE)
        self.assertIn('"livo_visual_input_contract"',SOURCE)

    def test_vendor_rejection_uses_frozen_cause_and_revokes_before_detailed_logging(self):
        for kind in ('points','imu','image','zupt'):
            self.assertIn('admit_'+kind+'_checked(',SOURCE)
            self.assertNotIn('vendor_'+kind+'_admission_rejected',SOURCE)
        failure=SOURCE.split('  void vendor_fault(',1)[1].split('  bool vendor_admission(',1)[0]
        self.assertLess(failure.index('fault(detail::vendor_fault_cause(info))'),
                        failure.index('info.describe()'))
        self.assertIn('LIVO_FIRST_FAULT_DETAIL',failure)
        reason=SOURCE.split('std::string NavigationRuntime::failure_reason()const',1)[1].split('std::shared_ptr<tf2_ros::Buffer>',1)[0]
        self.assertIn('detail::vendor_owner_failure_reason(*impl_->vendor_owner)',reason)
        self.assertNotIn('livo_vendor_owner_failed',reason)
        self.assertIn('if (navigation->failed()) revoke("navigation:" + navigation->failure_reason())',MAIN)

    def test_original_time_and_actual_owner_identity_are_preserved(self):
        self.assertIn('ns(input->color->header.stamp)', SOURCE)
        self.assertIn('input->first_sdk_callback_ros_ns', SOURCE)
        self.assertIn('input->first_callback_steady_ns', SOURCE)
        self.assertIn('get_fully_qualified_name()', SOURCE)
        self.assertNotIn('rslidar_points_destination_0', SOURCE)
        self.assertIn('airy_adapter->receive_points(input->cloud', SOURCE)
        self.assertIn('airy_adapter->receive_imu(input->imu', SOURCE)
        self.assertIn('ndt_owner->submit_points(message,ndt_stamp(delivery,stamp.source_identity))', SOURCE)
        self.assertIn('sinks.airy_frame(message,stamp)', SOURCE)
        self.assertIn('ndt_owner->submit_prior(message', SOURCE)
        self.assertIn('sinks.livo_candidate(std::move(message),stamp)', SOURCE)
        self.assertNotIn('use_propagated_odometry', SOURCE)
        self.assertNotIn('world_cloud->', SOURCE)

    def test_direct_transforms_are_configured_before_joint_or_camera_input(self):
        self.assertLess(SOURCE.index('configure_direct_transforms(transforms,'),
                        SOURCE.index('perception::make_joint_ingress'))
        self.assertLess(SOURCE.index('configure_direct_transforms(transforms,'),
                        SOURCE.index('camera::create_camera_runtime('))
        self.assertIn('direct_tf_buffer() const', HEADER)
        self.assertIn('real perception_output/Nav2 consumer required before start', SOURCE)
        self.assertIn('"publish_body_clouds",options.debug_publish_body_clouds', SOURCE)

    def test_canonical_manager_keeps_raw_tracking_and_correction_distinct(self):
        self.assertEqual(SOURCE.count('manager::make_localization_direct_runtime('), 1)
        candidate = SOURCE.split('  void receive_manager(LivoCandidate', 1)[1].split('  void receive_manager(NdtDelivery', 1)[0]
        self.assertIn('receive_odometry("livo",input.message', candidate)
        correction = SOURCE.split('  void receive_manager(NdtDelivery delivery)', 1)[1].split('  void receive_manager(BackendStatus', 1)[0]
        self.assertIn('if(livo_alignment_enabled)manager_owner->receive_livo_ndt_alignment', correction)
        self.assertIn('if(input->independent_tracking_evidence)manager_owner->receive_tracking', correction)
        self.assertIn('map.source_ns=ns(input->map_pose->header.stamp)', correction)
        canonical = SOURCE.split('  void receive_canonical(std::shared_ptr<const manager::CanonicalLocalizationState>', 1)[1].split('  void receive_canonical(', 1)[0]
        self.assertLess(canonical.index('setTransform(state->map_to_odom'), canonical.index('sinks.canonical_localization'))
        self.assertIn('setTransform(state->odom_to_base_footprint', canonical)
        self.assertIn('kinematics_options(options.perception_node,options.kinematics_tf_mode)', SOURCE)
        self.assertIn('kinematics_options(options.joint_node,options.kinematics_tf_mode)', SOURCE)
        self.assertIn('mode!="production_direct"&&mode!="attended_direct"', SOURCE)
        self.assertIn('required_text(*entry, "kinematics_tf_mode")', MAIN)

    def test_control_and_joint_executors_construct_independent_strategies(self):
        self.assertIn('MultiThreadedExecutor>(detail::make_executor_options(context),4)', SOURCE)
        self.assertIn('SingleThreadedExecutor>(detail::make_executor_options(joint_context))', SOURCE)
        self.assertIn('const auto joint_context=joints->get_node_base_interface()->get_context()', SOURCE)
        self.assertIn('detail::require_joint_context_domain(context,joint_context)', SOURCE)
        self.assertIn('detail::spin_joint_executor(*joint_executor,context', SOURCE)
        self.assertNotIn('ExecutorOptions executor_options;', SOURCE)

    def test_source_shutdown_precedes_processing_owner_shutdown(self):
        stop = SOURCE.split('  void stop_locked(){', 1)[1]
        self.assertLess(stop.index('airy_owner->stop()'), stop.index('point_lane->join()'))
        self.assertLess(stop.index('stop_camera_runtime(camera_owner)'), stop.index('image_lane->join()'))
        self.assertLess(stop.index('point_lane->join()'), stop.index('vendor_owner->stop()'))
        self.assertLess(stop.index('vendor_owner->stop()'), stop.index('vendor_output_lane->join()'))
        self.assertLess(stop.index('vendor_output_lane->join()'), stop.index('ndt_owner->stop()'))
        self.assertLess(stop.index('control_thread.join()'), stop.index('perception_owner->stop()'))

    def test_executable_binds_real_consumers_and_preserves_safety_observers(self):
        self.assertLess(MAIN.index('assembly->start();'), MAIN.index('navigation->start();'))
        self.assertIn('owner->consume_body(std::move(output))', MAIN)
        self.assertIn('owner->consume_localization(std::move(state))', MAIN)
        self.assertIn('owner->consume_health(std::move(health))', MAIN)
        self.assertIn('options.debug_publish_body_clouds = false', MAIN)
        self.assertIn('nav2_options.publish_filtered_airy_compatibility = false', MAIN)
        self.assertIn('owner->admit_filtered_airy(std::move(output))', MAIN)
        self.assertIn('make_coverage_direct_runtime', SOURCE)
        self.assertIn('coverage_owner->receive_spatial(std::move(evidence))', SOURCE)
        self.assertIn('options.publish_ros_canonical_compatibility = true', MAIN)
        self.assertIn('localization_manager()->revoke(reason)', MAIN)
        self.assertIn('startup_deadline_waiting_for_original_localization_tf_map_or_sensor_gates', MAIN)
        self.assertNotIn('create_publisher', MAIN)
        self.assertNotIn('create_subscription', MAIN)

    def test_peripherals_require_one_real_response_with_the_original_deadline(self):
        self.assertIn('required_text(*entry, "nav2.peripheral_lifecycle_manager")', MAIN)
        self.assertIn('"/lifecycle_manager_map_prerequisites/is_active"', MAIN)
        for action in ('compute_path_to_pose', 'compute_path_through_poses', 'follow_path'):
            self.assertIn('entry,"'+action+'"', MAIN)
        self.assertIn('startup.advance(assembly->initialized(),ready,response,startup_poll_now)', MAIN)
        self.assertIn('startup.should_poll(assembly->motion_held(),startup_poll_now)', MAIN)
        self.assertIn('maps_active && peripherals->service_is_ready()', MAIN)
        self.assertEqual(MAIN.count('peripherals->async_send_request(request)'), 1)
        self.assertIn('startup_response.get()->success', MAIN)
        self.assertIn('options.defer_spatial_evidence_until_startup = true', MAIN)
        self.assertIn('navigation->release_spatial_startup(deadline_ns)', MAIN)
        self.assertIn('navigation->spatial_startup_status()', MAIN)
        self.assertIn('healthy_once = peripherals_complete && spatial.first_published_steady_ns > 0', MAIN)
        self.assertIn('spatial.first_published_steady_ns < deadline_ns', MAIN)
        self.assertIn('nav2_core_lost_waiting_for_first_spatial_evidence', MAIN)
        self.assertNotIn('healthy_once || assembly->active()', MAIN)
        self.assertIn('catch (const std::exception & error) {revoke(error.what());}', MAIN)


if __name__ == '__main__':
    unittest.main()
