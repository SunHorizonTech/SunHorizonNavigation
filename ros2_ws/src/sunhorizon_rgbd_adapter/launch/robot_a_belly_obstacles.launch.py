"""Independent, private-TF belly obstacle pipeline. No motion or localization."""
import copy
import hashlib
import json
import math
import os
import re
from pathlib import Path
import xml.etree.ElementTree as ET


def prepare(profile, urdf, serial):
    if profile['schema_version'] != 1 or profile['robot_serial'] != serial:
        raise ValueError('belly_profile_robot_mismatch')
    if hashlib.sha256(urdf).hexdigest() != profile['urdf_sha256']:
        raise ValueError('belly_profile_urdf_mismatch')
    root = ET.fromstring(urdf)
    names = profile['joint_names']
    if len(names) != len(set(names)) or len(names) != 12:
        raise ValueError('belly_profile_joint_set_invalid')
    joints = [j for j in root.findall('joint') if j.attrib['name'] in names]
    if {j.attrib['name'] for j in joints} != set(names) or len(joints) != len(names):
        raise ValueError('belly_profile_joints_missing')
    base = profile['base_frame']
    connected = {base}
    pending = list(joints)
    while pending:
        available = [j for j in pending if j.find('parent').attrib['link'] in connected]
        if not available:
            raise ValueError('belly_profile_joint_tree_disconnected')
        for joint in available:
            child = joint.find('child').attrib['link']
            if child in connected:
                raise ValueError('belly_profile_joint_tree_not_unique')
            connected.add(child)
            pending.remove(joint)
    model = ET.Element('robot', {'name': 'robot_a_belly_self_model'})
    for joint in joints:
        model.append(copy.deepcopy(joint))
    for link in sorted(connected):
        ET.SubElement(model, 'link', {'name': link})
    volumes = profile['volumes']
    if not 1 <= len(volumes) <= 8:
        raise ValueError('belly_profile_volume_count_invalid')
    for box in volumes:
        if box['frame'] not in connected - {base}:
            raise ValueError('belly_profile_volume_frame_invalid')
        lo, hi = box['minimum'], box['maximum']
        if len(lo) != 3 or len(hi) != 3 or any(
                not math.isfinite(a) or not math.isfinite(b) or not -2 <= a < b <= 2 or b-a > 1
                for a, b in zip(lo, hi)):
            raise ValueError('belly_profile_volume_bounds_invalid')
    camera = profile['camera_to_base']
    t, q = camera['translation'], camera['quaternion']
    if len(t) != 3 or len(q) != 4 or not all(math.isfinite(v) for v in t + q) or abs(sum(v*v for v in q)-1) > 1e-6:
        raise ValueError('belly_profile_camera_transform_invalid')
    if not math.isfinite(profile['joint_publish_frequency_hz']) or profile['joint_publish_frequency_hz'] <= 0:
        raise ValueError('belly_profile_joint_frequency_invalid')
    tf_namespace = profile['tf_namespace']
    if not tf_namespace.startswith('/sunhorizon/navigation/') or tf_namespace.endswith('/'):
        raise ValueError('belly_profile_tf_namespace_invalid')
    topics = [profile[k] for k in ('joint_state_topic', 'depth_topic', 'camera_info_topic', 'output_topic')]
    if len(set(topics)) != len(topics) or any(not v.startswith('/sunhorizon/') for v in topics):
        raise ValueError('belly_profile_topics_invalid')
    depth = dict(profile['depth_parameters'])
    depth.update(depth_topic=profile['depth_topic'], camera_info_topic=profile['camera_info_topic'],
                 output_topic=tf_namespace + '/components', diagnostics_topic=tf_namespace + '/depth_diagnostics')
    body = dict(profile['self_parameters'])
    body.update(input_topic=depth['output_topic'], output_topic=profile['output_topic'],
                diagnostics_topic=tf_namespace + '/body_diagnostics', base_frame=base)
    if body.get('self_exclusion_mode','aabb-v1')=='dynamic-mesh-v1':
        if (body.get('enabled') is not True or base!='base_link' or
                body.get('shadow_sensor_range_m')!=0. or tf_namespace!='/sunhorizon/navigation/belly_self' or
                profile['output_topic']!='/sunhorizon/navigation/belly/points'):
            raise ValueError('dynamic_self_requires_exclusive_belly_profile')
        body.update(exclusion_frames=[base],exclusion_min_xyz=[0.,0.,0.],exclusion_max_xyz=[0.,0.,0.],
                    additional_exclusion_min_xyz=[],additional_exclusion_max_xyz=[])
    else:
        if body.get('self_exclusion_mode','aabb-v1')!='aabb-v1':
            raise ValueError('unknown_self_exclusion_mode')
        body.update(exclusion_frames=[v['frame'] for v in volumes],
                exclusion_min_xyz=volumes[0]['minimum'], exclusion_max_xyz=volumes[0]['maximum'],
                additional_exclusion_min_xyz=[n for v in volumes[1:] for n in v['minimum']],
                additional_exclusion_max_xyz=[n for v in volumes[1:] for n in v['maximum']])
    return ET.tostring(model, encoding='unicode'), depth, body


def private_tf_environment():
    # Both a source package and an installed package share have sibling launch/
    # and config/ directories. Resolve this launch's own committed profile.
    profile = (Path(__file__).resolve().parents[1] / 'config/robot_a_fastdds.xml').resolve(strict=True)
    if not profile.is_file():
        raise ValueError('belly_private_tf_transport_profile_not_file')
    return {'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
            'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile)}


def spatial_sdk_environment(environ):
    # The service shell sources its original ROS installation before launch.
    # Dedicated values survive that setup and are applied to the three new
    # children afterwards. The owning transaction pins their exact SDK inputs.
    result = {}
    for key in ('LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH', 'PYTHONPATH'):
        value = environ.get('SUNHORIZON_SPATIAL_'+key)
        if not isinstance(value, str) or not value or '\n' in value or '\r' in value:
            raise ValueError('spatial_sdk_environment_missing_or_invalid:'+key)
        result[key] = value
    result['PYTHONDONTWRITEBYTECODE'] = '1'
    return result


def body_launch_parameters(body):
    # Humble launch_ros cannot evaluate an untyped empty sequence in a Node
    # parameter dictionary. These two pinned C++ parameters default to empty
    # vector<double>; the dynamic mode also rejects any nonempty override.
    result = dict(body)
    if body.get('self_exclusion_mode') == 'dynamic-mesh-v1':
        for key in ('additional_exclusion_min_xyz', 'additional_exclusion_max_xyz'):
            if type(result.get(key)) is not list or result[key] != []:
                raise ValueError('dynamic_self_empty_default_required:' + key)
            del result[key]
    return result


def validate_region_receipt(depth, receipt):
    """Old depth ELFs silently ignore overrides; pin tested opt-in capability."""
    enabled = depth.get('region_support_enabled', False)
    if type(enabled) is not bool:
        raise ValueError('region_support_enabled_must_be_bool')
    shadow = depth.get('region_shadow_only_enabled', False)
    if type(shadow) is not bool:
        raise ValueError('region_shadow_only_must_be_bool')
    if shadow and not enabled:
        raise ValueError('region_shadow_only_requires_enabled_region')
    hold = depth.get('region_hold_complete_measurement_enabled', False)
    if type(hold) is not bool:
        raise ValueError('region_hold_complete_must_be_bool')
    if hold and not enabled:
        raise ValueError('region_hold_complete_requires_enabled_region')
    wait = depth.get('region_tf_wait_sec', 0.)
    if type(wait) not in (int, float) or not 0 <= wait <= .02 or not math.isfinite(wait):
        raise ValueError('region_tf_wait_outside_envelope')
    if not enabled:
        if wait != 0:
            raise ValueError('region_tf_wait_requires_enabled_region')
        return
    capabilities = receipt.get('capabilities')
    if (type(receipt.get('schema_version')) is not int or receipt.get('schema_version') != 1 or
            receipt.get('package') != 'sunhorizon_rgbd_adapter' or
            not isinstance(receipt.get('source_commit'), str) or
            not re.fullmatch('[0-9a-f]{40}', receipt['source_commit']) or
            type(capabilities) is not list or any(type(value) is not str for value in capabilities) or
            'region_depth_support_exact_pair_v1' not in capabilities):
        raise ValueError('region_depth_artifact_capability_missing')
    if wait != 0 and 'region_depth_support_bounded_tf_wait_v1' not in capabilities:
        raise ValueError('region_tf_wait_artifact_capability_missing')
    if shadow and 'region_depth_support_shadow_rays_v1' not in capabilities:
        raise ValueError('region_shadow_only_artifact_capability_missing')
    if hold and 'region_depth_support_hold_complete_v1' not in capabilities:
        raise ValueError('region_hold_complete_artifact_capability_missing')
    for key in ('region_expected_contract', 'region_expected_calibration',
                'region_support_diagnostics_topic'):
        if not isinstance(depth.get(key), str) or not depth[key]:
            raise ValueError('region_depth_required_pin_missing:' + key)
    for key in ('region_rectified_pinhole_accepted', 'region_metric_axial_depth_accepted',
                'region_depth_intrinsics_registration_accepted',
                'region_optical_to_base_extrinsics_accepted', 'region_source_time_transform_accepted'):
        if type(depth.get(key)) is not bool:
            raise ValueError('region_depth_acceptance_must_be_explicit_bool:' + key)
        if shadow and depth[key]:
            raise ValueError('region_shadow_only_requires_all_unaccepted:' + key)


def launch_pipeline(context):
    from launch.actions import EmitEvent, GroupAction, RegisterEventHandler, UnsetEnvironmentVariable
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue
    profile = json.loads(Path(LaunchConfiguration('profile').perform(context)).read_text())
    urdf = Path(LaunchConfiguration('vendor_urdf').perform(context)).read_bytes()
    serial = Path('/sys/firmware/devicetree/base/serial-number').read_bytes().replace(b'\0', b'').decode().strip()
    model, depth, body = prepare(profile, urdf, serial)
    prefix = Path(LaunchConfiguration('filter_install').perform(context)).resolve()
    receipt = json.loads((prefix / 'artifact-receipt.json').read_text())
    validate_region_receipt(depth, receipt)
    if body.get('self_exclusion_mode')=='dynamic-mesh-v1' and 'dynamic_self_mesh_v1' not in receipt.get('capabilities',[]):
        raise ValueError('dynamic_self_artifact_capability_missing')
    binaries = {}
    spatial=profile.get('spatial_parameters')
    if spatial is not None and (body.get('self_exclusion_mode')!='dynamic-mesh-v1' or
            type(spatial) is not dict or spatial.get('enabled') is not True or
            'spatial_visibility_evidence_v1' not in receipt.get('capabilities',[])):
        raise ValueError('spatial_belly_launch_contract_invalid')
    for name in ('depth_obstacle_cloud_node', 'pointcloud_self_exclusion_node')+(
            ('spatial_visibility_node',) if spatial is not None else ()):
        binary = prefix / 'lib/sunhorizon_rgbd_adapter' / name
        if hashlib.sha256(binary.read_bytes()).hexdigest() != receipt['binary_sha256'][name]:
            raise ValueError('belly_filter_binary_hash_mismatch:' + name)
        binaries[name] = str(binary)
    private_tf = [('/tf', profile['tf_namespace'] + '/tf'),
                  ('/tf_static', profile['tf_namespace'] + '/tf_static')]
    camera = profile['camera_to_base']
    arguments = [str(v) for v in camera['translation'] + camera['quaternion']]
    tf_environment = private_tf_environment()
    processing_environment = spatial_sdk_environment(os.environ) if spatial is not None else {}
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='sunhorizon_belly_leg_model', output='screen',
             additional_env=tf_environment,
             parameters=[{'robot_description': ParameterValue(model, value_type=str),
                          'publish_frequency': profile['joint_publish_frequency_hz']}],
             remappings=[*private_tf, ('joint_states', profile['joint_state_topic'])]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='sunhorizon_belly_model_camera',
             additional_env=tf_environment,
             arguments=arguments + [profile['base_frame'], profile['sensor_frame']],
             remappings=private_tf, output='screen'),
        Node(executable=binaries['depth_obstacle_cloud_node'], name='sunhorizon_belly_obstacle_depth',
             additional_env=processing_environment, parameters=[depth], output='screen'),
        Node(executable=binaries['pointcloud_self_exclusion_node'], name='sunhorizon_belly_self_filter',
             additional_env=processing_environment, parameters=[body_launch_parameters(body)], remappings=private_tf, output='screen')]
    if spatial is not None:
        # It has independent global/private TF buffers. Do not inherit the
        # legacy body's /tf remapping or replace the original depth producer.
        nodes.append(Node(executable=binaries['spatial_visibility_node'],name='sunhorizon_spatial_visibility',
                          additional_env=processing_environment,parameters=[spatial],output='screen'))
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
        EmitEvent(event=Shutdown(reason='belly_obstacle_pipeline_child_exited'))])) for node in nodes]
    # Limit the SHM wait workaround to the two small private-TF publishers.
    # The scoped unset cannot leak into the depth/body processing environment.
    private_tf_group = GroupAction(actions=[
        UnsetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS'), *nodes[:2]], scoped=True)
    return [private_tf_group, *nodes[2:], *handlers]


def generate_launch_description():
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument, OpaqueFunction
    return LaunchDescription([
        DeclareLaunchArgument('profile', description='Versioned measured obstacle profile JSON'),
        DeclareLaunchArgument('vendor_urdf', description='Original vendor URDF matching the profile hash'),
        DeclareLaunchArgument('filter_install', description='Tested filter prefix with artifact-receipt.json'),
        OpaqueFunction(function=launch_pipeline)])
