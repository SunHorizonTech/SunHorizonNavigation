import copy
import hashlib
import importlib.util
import json
import sys
import types
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('belly_launch', PACKAGE / 'launch/robot_a_belly_obstacles.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_region_opt_in_requires_tested_binary_capability_and_explicit_contract():
    MODULE.validate_region_receipt({}, {})  # Legacy/default path remains unchanged.
    with pytest.raises(ValueError, match='must_be_bool'):
        MODULE.validate_region_receipt({'region_support_enabled': 'true'}, {})
    depth = dict(region_support_enabled=True, region_expected_contract='pinned',
                 region_expected_calibration='pinned-info',
                 region_support_diagnostics_topic='/sunhorizon/region/support')
    for key in ('region_rectified_pinhole_accepted', 'region_metric_axial_depth_accepted',
                'region_depth_intrinsics_registration_accepted',
                'region_optical_to_base_extrinsics_accepted', 'region_source_time_transform_accepted'):
        depth[key] = False
    with pytest.raises(ValueError, match='capability_missing'):
        MODULE.validate_region_receipt(depth, {})
    receipt = dict(schema_version=1, package='sunhorizon_rgbd_adapter',
                   source_commit='a' * 40, capabilities=['region_depth_support_exact_pair_v1'])
    MODULE.validate_region_receipt(depth, receipt)  # False attestations are valid shadow input.
    for capabilities in ('not_region_depth_support_exact_pair_v1_accepted',
                         {'region_depth_support_exact_pair_v1': True},
                         ['region_depth_support_exact_pair_v1', 1], None):
        invalid = {**receipt, 'capabilities': capabilities}
        with pytest.raises(ValueError, match='capability_missing'):
            MODULE.validate_region_receipt(depth, invalid)
    with pytest.raises(ValueError, match='capability_missing'):
        MODULE.validate_region_receipt(depth, {**receipt, 'schema_version': True})
    for key in ('region_expected_contract', 'region_expected_calibration',
                'region_support_diagnostics_topic', 'region_metric_axial_depth_accepted'):
        invalid = copy.deepcopy(depth)
        del invalid[key]
        with pytest.raises(ValueError):
            MODULE.validate_region_receipt(invalid, receipt)


def test_nonzero_region_wait_requires_both_exact_capabilities_and_bounded_type():
    depth = dict(region_support_enabled=True, region_expected_contract='pinned',
                 region_expected_calibration='pinned-info', region_support_diagnostics_topic='/sunhorizon/region/support')
    for key in ('region_rectified_pinhole_accepted', 'region_metric_axial_depth_accepted',
                'region_depth_intrinsics_registration_accepted',
                'region_optical_to_base_extrinsics_accepted', 'region_source_time_transform_accepted'):
        depth[key] = False
    receipt = dict(schema_version=1, package='sunhorizon_rgbd_adapter', source_commit='a' * 40,
                   capabilities=['region_depth_support_exact_pair_v1'])
    for zero in (0, 0., -0.):
        MODULE.validate_region_receipt({**depth, 'region_tf_wait_sec': zero}, receipt)
    waited = {**depth, 'region_tf_wait_sec': .02}
    with pytest.raises(ValueError, match='wait_artifact_capability_missing'):
        MODULE.validate_region_receipt(waited, receipt)
    both = {**receipt, 'capabilities': [*receipt['capabilities'], 'region_depth_support_bounded_tf_wait_v1']}
    MODULE.validate_region_receipt(waited, both)
    MODULE.validate_region_receipt({**depth, 'region_tf_wait_sec': .000001}, both)
    for capabilities in (['region_depth_support_bounded_tf_wait_v1'],
            ['region_depth_support_exact_pair_v1', 'not_region_depth_support_bounded_tf_wait_v1'],
            'region_depth_support_exact_pair_v1 region_depth_support_bounded_tf_wait_v1',
            {'region_depth_support_exact_pair_v1': True, 'region_depth_support_bounded_tf_wait_v1': True}):
        with pytest.raises(ValueError, match='capability_missing'):
            MODULE.validate_region_receipt(waited, {**receipt, 'capabilities': capabilities})
    for invalid in (True, False, -.000001, .0200000001, float('inf'), float('nan'), '0.02', None):
        with pytest.raises(ValueError, match='wait_outside_envelope'):
            MODULE.validate_region_receipt({**depth, 'region_tf_wait_sec': invalid}, both)
    with pytest.raises(ValueError, match='requires_enabled_region'):
        MODULE.validate_region_receipt({'region_tf_wait_sec': .02}, both)


def test_shadow_only_requires_strict_bool_all_false_and_exact_capabilities():
    acceptance = ('region_rectified_pinhole_accepted', 'region_metric_axial_depth_accepted',
        'region_depth_intrinsics_registration_accepted', 'region_optical_to_base_extrinsics_accepted',
        'region_source_time_transform_accepted')
    depth = dict(region_support_enabled=True, region_expected_contract='pinned',
        region_expected_calibration='pinned-info', region_support_diagnostics_topic='/sunhorizon/region/support',
        **{key: False for key in acceptance})
    exact = 'region_depth_support_exact_pair_v1'
    wait = 'region_depth_support_bounded_tf_wait_v1'
    shadow = 'region_depth_support_shadow_rays_v1'
    receipt = dict(schema_version=1, package='sunhorizon_rgbd_adapter', source_commit='a' * 40,
        capabilities=[exact])
    MODULE.validate_region_receipt({**depth, 'region_shadow_only_enabled': False}, receipt)
    for invalid in (0, 1, 'true', None, [], {}):
        with pytest.raises(ValueError, match='shadow_only_must_be_bool'):
            MODULE.validate_region_receipt({**depth, 'region_shadow_only_enabled': invalid}, receipt)
    with pytest.raises(ValueError, match='requires_enabled_region'):
        MODULE.validate_region_receipt({'region_shadow_only_enabled': True}, receipt)
    diagnostic = {**depth, 'region_shadow_only_enabled': True}
    for capabilities in ([exact], [shadow], [exact, 'not_' + shadow], {exact: True, shadow: True}, exact + shadow):
        with pytest.raises(ValueError, match='capability_missing'):
            MODULE.validate_region_receipt(diagnostic, {**receipt, 'capabilities': capabilities})
    both = {**receipt, 'capabilities': [exact, shadow]}
    MODULE.validate_region_receipt(diagnostic, both)
    for key in acceptance:
        with pytest.raises(ValueError, match='all_unaccepted'):
            MODULE.validate_region_receipt({**diagnostic, key: True}, both)
    with pytest.raises(ValueError, match='wait_artifact_capability_missing'):
        MODULE.validate_region_receipt({**diagnostic, 'region_tf_wait_sec': .02}, both)
    MODULE.validate_region_receipt({**diagnostic, 'region_tf_wait_sec': .02},
        {**receipt, 'capabilities': [exact, wait, shadow]})


def test_hold_complete_requires_typed_opt_in_and_capability():
    flags = ('rectified_pinhole', 'metric_axial_depth', 'depth_intrinsics_registration',
             'optical_to_base_extrinsics', 'source_time_transform')
    depth = dict(region_support_enabled=True, region_expected_contract='pinned',
        region_expected_calibration='pinned-info', region_support_diagnostics_topic='/sunhorizon/region/support',
        **{'region_' + key + '_accepted': True for key in flags})
    receipt = dict(schema_version=1, package='sunhorizon_rgbd_adapter', source_commit='a' * 40,
        capabilities=['region_depth_support_exact_pair_v1'])
    MODULE.validate_region_receipt({**depth, 'region_hold_complete_measurement_enabled': False}, receipt)
    for invalid in (0, 1, 'true', None, [], {}):
        with pytest.raises(ValueError, match='hold_complete_must_be_bool'):
            MODULE.validate_region_receipt({**depth, 'region_hold_complete_measurement_enabled': invalid}, receipt)
    with pytest.raises(ValueError, match='requires_enabled_region'):
        MODULE.validate_region_receipt({'region_hold_complete_measurement_enabled': True}, receipt)
    selected = {**depth, 'region_hold_complete_measurement_enabled': True}
    with pytest.raises(ValueError, match='hold_complete_artifact_capability_missing'):
        MODULE.validate_region_receipt(selected, receipt)
    MODULE.validate_region_receipt(selected, {**receipt, 'capabilities': [*receipt['capabilities'],
        'region_depth_support_hold_complete_v1']})


def fixture():
    profile = json.loads((PACKAGE / 'config/robot_a_belly_obstacles.prototype.json').read_text())
    robot = ET.Element('robot', {'name': 'fixture'})
    for side in ('left', 'right'):
        parent = 'base_link'
        for name in [n for n in profile['joint_names'] if n.startswith(side)]:
            joint = ET.SubElement(robot, 'joint', {'name': name, 'type': 'revolute'})
            child = name.removesuffix('_joint') + '_link'
            ET.SubElement(joint, 'parent', {'link': parent})
            ET.SubElement(joint, 'child', {'link': child})
            ET.SubElement(joint, 'origin', {'xyz': '.01 .02 .03', 'rpy': '.1 .2 .3'})
            ET.SubElement(joint, 'axis', {'xyz': '0 1 0'})
            ET.SubElement(joint, 'limit', {'lower': '-1', 'upper': '1', 'effort': '2', 'velocity': '3'})
            parent = child
    urdf = ET.tostring(robot)
    profile['urdf_sha256'] = hashlib.sha256(urdf).hexdigest()
    return profile, urdf


def test_leg_model_preserves_joint_geometry_and_isolates_outputs():
    p, urdf = fixture()
    model, depth, body = MODULE.prepare(p, urdf, p['robot_serial'])
    original, generated = ET.fromstring(urdf), ET.fromstring(model)
    assert len(generated.findall('link')) == 13
    assert [ET.tostring(j) for j in original.findall('joint')] == [ET.tostring(j) for j in generated.findall('joint')]
    assert depth['output_topic'] == body['input_topic']
    assert body['output_topic'] == p['output_topic']
    assert len(body['exclusion_frames']) == 8
    assert depth['component_filter_max_depth_m'] == body['shadow_sensor_range_m'] == 1.
    assert depth['max_cloud_age_sec'] == body['max_cloud_age_sec'] == .3


def test_mismatched_robot_model_and_disconnected_geometry_fail_before_launch():
    p, urdf = fixture()
    with pytest.raises(ValueError, match='robot_mismatch'):
        MODULE.prepare(p, urdf, 'wrong_robot')
    with pytest.raises(ValueError, match='urdf_mismatch'):
        MODULE.prepare(p, urdf + b' ', p['robot_serial'])
    wrong = copy.deepcopy(p)
    wrong['volumes'][0]['frame'] = 'not_a_leg'
    with pytest.raises(ValueError, match='volume_frame_invalid'):
        MODULE.prepare(wrong, urdf, p['robot_serial'])
    wrong = copy.deepcopy(p)
    wrong['tf_namespace'] = '/tf'
    with pytest.raises(ValueError, match='tf_namespace_invalid'):
        MODULE.prepare(wrong, urdf, p['robot_serial'])
    tree = ET.fromstring(urdf)
    tree.find('joint/parent').set('link', 'missing_parent')
    disconnected = ET.tostring(tree)
    p['urdf_sha256'] = hashlib.sha256(disconnected).hexdigest()
    with pytest.raises(ValueError, match='tree_disconnected'):
        MODULE.prepare(p, disconnected, p['robot_serial'])


def test_private_tf_profile_resolves_source_and_installed_share(tmp_path, monkeypatch):
    source = Path(MODULE.private_tf_environment()['FASTRTPS_DEFAULT_PROFILES_FILE'])
    ns = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}
    root = ET.parse(source).getroot()
    assert root.findtext('.//f:useBuiltinTransports', namespaces=ns) == 'false'
    assert [node.text for node in root.findall('.//f:transport_descriptor/f:type', ns)] == ['UDPv4']
    share = tmp_path / 'share/sunhorizon_rgbd_adapter'
    (share / 'launch').mkdir(parents=True)
    monkeypatch.setattr(MODULE, '__file__', str(share / 'launch/robot_a_belly_obstacles.launch.py'))
    with pytest.raises(FileNotFoundError):
        MODULE.private_tf_environment()
    (share / 'config').mkdir()
    installed = share / 'config/robot_a_fastdds.xml'
    installed.write_bytes(source.read_bytes())
    assert MODULE.private_tf_environment() == {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(installed.resolve())}


@pytest.mark.parametrize('region_enabled', [False, True])
def test_only_two_private_tf_nodes_receive_scoped_udp_transport(tmp_path, monkeypatch, region_enabled):
    def action(kind):
        return lambda *args, **kwargs: types.SimpleNamespace(kind=kind, args=args, kwargs=kwargs)
    class Configuration:
        def __init__(self, name): self.name = name
        def perform(self, context): return str(context[self.name])
    modules = {
        'launch.actions': dict(EmitEvent=action('emit'), GroupAction=action('group'),
                               RegisterEventHandler=action('handler'),
                               UnsetEnvironmentVariable=action('unset')),
        'launch.event_handlers': dict(OnProcessExit=action('exit')),
        'launch.events': dict(Shutdown=action('shutdown')),
        'launch.substitutions': dict(LaunchConfiguration=Configuration),
        'launch_ros.actions': dict(Node=action('node')),
        'launch_ros.parameter_descriptions': dict(ParameterValue=action('parameter')),
    }
    for name, fields in modules.items():
        module = types.ModuleType(name)
        module.__dict__.update(fields)
        monkeypatch.setitem(sys.modules, name, module)
    p, urdf = fixture()
    if region_enabled:
        p['depth_parameters'].update(region_support_enabled=True,
            region_expected_contract='fixed-contract', region_expected_calibration='fixed-info',
            region_support_diagnostics_topic='/sunhorizon/region/support',
            region_tf_wait_sec=.02, region_hold_complete_measurement_enabled=True,
            **{'region_' + key + '_accepted': True for key in (
                'rectified_pinhole', 'metric_axial_depth', 'depth_intrinsics_registration',
                'optical_to_base_extrinsics', 'source_time_transform')})
    profile_file, urdf_file = tmp_path / 'profile.json', tmp_path / 'robot.urdf'
    profile_file.write_text(json.dumps(p))
    urdf_file.write_bytes(urdf)
    prefix = tmp_path / 'install'
    binary_dir = prefix / 'lib/sunhorizon_rgbd_adapter'
    binary_dir.mkdir(parents=True)
    hashes = {}
    for name in ('depth_obstacle_cloud_node', 'pointcloud_self_exclusion_node'):
        (binary_dir / name).write_bytes(name.encode())
        hashes[name] = hashlib.sha256(name.encode()).hexdigest()
    receipt = dict(binary_sha256=hashes, schema_version=1,
        package='sunhorizon_rgbd_adapter', source_commit='a' * 40,
        capabilities=['region_depth_support_exact_pair_v1',
            'region_depth_support_bounded_tf_wait_v1', 'region_depth_support_hold_complete_v1'])
    (prefix / 'artifact-receipt.json').write_text(json.dumps(receipt))
    read_bytes = Path.read_bytes
    monkeypatch.setattr(Path, 'read_bytes', lambda path:
        p['robot_serial'].encode() + b'\0' if str(path) == '/sys/firmware/devicetree/base/serial-number'
        else read_bytes(path))
    launched = MODULE.launch_pipeline({'profile': profile_file, 'vendor_urdf': urdf_file,
                                      'filter_install': prefix})
    group = launched[0]
    assert group.kind == 'group' and group.kwargs['scoped'] is True
    unset, model, camera = group.kwargs['actions']
    assert unset.kind == 'unset' and unset.args == ('FASTDDS_BUILTIN_TRANSPORTS',)
    assert [node.kwargs['executable'] for node in (model, camera)] == [
        'robot_state_publisher', 'static_transform_publisher']
    assert model.kwargs['additional_env'] == camera.kwargs['additional_env'] == MODULE.private_tf_environment()
    depth, body = launched[1:3]
    assert depth.kwargs.get('additional_env', {}) == body.kwargs.get('additional_env', {}) == {}
    assert model.kwargs['parameters'][0]['publish_frequency'] == p['joint_publish_frequency_hz']
    assert camera.kwargs['arguments'] == [str(v) for v in
        p['camera_to_base']['translation'] + p['camera_to_base']['quaternion']] + [p['base_frame'], p['sensor_frame']]
    _, expected_depth, expected_body = MODULE.prepare(p, urdf, p['robot_serial'])
    assert depth.kwargs['parameters'] == [expected_depth]
    assert body.kwargs['parameters'] == [expected_body]

    # The four process actions retain all inherited production DDS/loader keys;
    # only the scoped TF group removes BUILTIN_TRANSPORTS and its two nodes add
    # their explicit transport profile. Depth/body remain outside that scope.
    inherited = dict(FASTDDS_BUILTIN_TRANSPORTS='LARGE_DATA',
        FASTRTPS_DEFAULT_PROFILES_FILE='/production/depth.xml',
        LD_LIBRARY_PATH='/production/dds-health', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    tf_environment = {key: value for key, value in inherited.items()
                      if key != unset.args[0]}
    for node in (model, camera):
        effective = {**tf_environment, **node.kwargs['additional_env']}
        assert 'FASTDDS_BUILTIN_TRANSPORTS' not in effective
        assert effective['LD_LIBRARY_PATH'] == inherited['LD_LIBRARY_PATH']
        assert effective['FASTRTPS_DEFAULT_PROFILES_FILE'] == MODULE.private_tf_environment()['FASTRTPS_DEFAULT_PROFILES_FILE']
    for node in (depth, body):
        assert {**inherited, **node.kwargs.get('additional_env', {})} == inherited
    if region_enabled:
        # The restored transport structure must not bypass the new capability gate.
        receipt['capabilities'] = []
        (prefix / 'artifact-receipt.json').write_text(json.dumps(receipt))
        with pytest.raises(ValueError, match='capability_missing'):
            MODULE.launch_pipeline({'profile': profile_file, 'vendor_urdf': urdf_file,
                                    'filter_install': prefix})
