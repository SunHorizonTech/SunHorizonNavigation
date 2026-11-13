"""Actual localhost ROS admission and typed safety gates for explicit raw350."""
import os
import subprocess

import pytest
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from test_typed_region_coverage_node import Harness, command, params, stamp_ns

PROFILE = 'raw-status-350ms-v1'
OPT_IN = {'region_upstream_status_deadline_profile': PROFILE, 'belly_freshness_sec': '.35',
          'chest_freshness_sec': '1.2'}


def this_phase(h, decision):
    return h.phase_first_input_ns is not None and stamp_ns(decision.coverage.header.stamp) >= h.phase_first_input_ns


@pytest.mark.parametrize('change', [
    {'region_upstream_status_deadline_profile': 'unknown'},
    {'region_upstream_status_deadline_profile': 'strict-300ms-v1'},
    {'region_support_required': 'false', 'region_typed_evaluation_required': 'false'},
    {'region_typed_evaluation_required': 'false'},
    {'profile': 'visual'},
    {'belly_obstacle_points_topic': "''"},
    {'belly_freshness_sec': '.349'},
    {'belly_freshness_sec': '.351'},
    {'belly_obstacle_freshness_sec': '.299'},
    {'belly_obstacle_freshness_sec': '.301'},
])
def test_wrong_profile_or_opt_in_combination_rejected(tmp_path, change):
    values = params()
    values.update(OPT_IN)
    values.update(change)
    environment = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID=str(208 + os.getpid() % 8))
    result = subprocess.run(command(values), env=environment, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, timeout=8)
    print(result.stdout)
    assert result.returncode != 0
    assert any(reason in result.stdout for reason in (
        'unknown region_upstream_status_deadline_profile', 'raw-status-350ms-v1 requires',
        'region support requires', 'typed region evidence requires'))


def test_actual_opt_in_preserves_raw_processed_and_typed_region_deadlines(tmp_path):
    h = Harness(tmp_path, True, OPT_IN)
    try:
        # Same numeric raw age as run1, with fresh accepted processed/region data.
        # Acceptance here is a synthetic decision, never field/safety acceptance.
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy,
                raw_age=.313944817, timeout=12.)
        client = h.node.create_client(SetParameters, '/sunhorizon_obstacle_coverage/set_parameters')
        assert client.wait_for_service(timeout_sec=3.)
        request = SetParameters.Request(parameters=[Parameter(
            name='region_upstream_status_deadline_profile',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='strict-300ms-v1'))])
        future = client.call_async(request)
        import time
        deadline = time.monotonic() + 3.
        while not future.done() and time.monotonic() < deadline:
            h.pump(raw_age=.313944817)
        assert future.done() and not future.result().results[0].successful
        h.node.destroy_client(client)
        fault = h.until(lambda d: this_phase(h, d) and not d.non_region_healthy and d.disposition == 0
                        and d.region_evaluation_complete and d.coverage.reason == 'belly_queue_delay',
                        raw_age=.351)
        assert not fault.coverage.healthy
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        # Leave the actual on_rgbd receipt cached until both original timelines
        # age out; it is not a newly supplied or renewed receipt timestamp.
        before = h.node.get_clock().now().nanoseconds
        fault = h.until(lambda d: this_phase(h, d) and not d.non_region_healthy and d.disposition == 0
                        and d.region_evaluation_complete and d.coverage.reason in ('belly_stale', 'belly_receipt_stale')
                        and d.coverage.header.stamp.sec * 10**9 + d.coverage.header.stamp.nanosec
                        > before + 350_000_000, send_rgbd=False)
        assert not fault.coverage.healthy
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        fault = h.until(lambda d: this_phase(h, d) and not d.non_region_healthy and d.disposition == 0
                        and d.region_evaluation_complete and d.coverage.reason == 'belly_unhealthy',
                        processed_age=.301)
        assert not fault.coverage.healthy
        h.until(lambda d: d.disposition == 1 and d.coverage.healthy)
        # Original region node receipt/source deadlines expire with no new R.
        fault = h.until(lambda d: this_phase(h, d) and d.non_region_healthy and not d.region_evaluation_complete
                        and d.disposition == 0
                        and d.coverage.reason == 'ok;region_support:region_stale'
                        and d.region_source_stamp_ns > 0
                        and stamp_ns(d.coverage.header.stamp) - d.region_source_stamp_ns > 300_000_000,
                        send_region=False)
        assert not fault.coverage.healthy
    finally:
        h.close()


def test_actual_opt_in_rejects_old_region_source_from_first_measurement(tmp_path):
    # Start with an old source instead of rolling a previously accepted source
    # backwards, which would intentionally latch a separate protocol fault.
    h = Harness(tmp_path, True, OPT_IN)
    try:
        fault = h.until(lambda d: this_phase(h, d) and d.non_region_healthy and not d.region_evaluation_complete
                        and d.disposition == 0 and d.coverage.reason == 'ok;region_support:region_stale'
                        and d.region_source_stamp_ns > 0
                        and 300_000_000 < stamp_ns(d.coverage.header.stamp) - d.region_source_stamp_ns <= 350_000_000,
                        region_source_age=.301, timeout=12.)
        assert not fault.coverage.healthy
    finally:
        h.close()
