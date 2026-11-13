"""Real localhost ROS cloud/TF integration; no production service or actuator."""
import hashlib
import os
import signal
import struct
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

INPUT='/sunhorizon/navigation/belly_self/components'
OUTPUT='/sunhorizon/navigation/belly/points'
PRIVATE='/sunhorizon/navigation/belly_self/tf'
DIAG='/dynamic_filter_test/diagnostics'

def spin(node, seconds, done=lambda: False):
    end=time.monotonic()+seconds
    while time.monotonic()<end and not done():rclpy.spin_once(node,timeout_sec=.002)

@pytest.fixture(params=['exact-gid-v1','startup-pin-once-v1'])
def runtime(tmp_path,request):
    os.environ['ROS_LOCALHOST_ONLY']='1'
    os.environ['ROS_DOMAIN_ID']=str(200+os.getpid()%20)
    rclpy.init(args=[])
    node=rclpy.create_node('dynamic_filter_test_source')
    tfpub=node.create_publisher(TFMessage,PRIVATE,QoSProfile(depth=100,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE))
    source=node.create_publisher(PointCloud2,INPUT,10)
    outputs=[];diags=[]
    sub=node.create_subscription(PointCloud2,OUTPUT,outputs.append,10)
    diagnostic=node.create_subscription(DiagnosticArray,DIAG,diags.append,10)
    spin(node,5,lambda:len(node.get_publishers_info_by_topic(PRIVATE))==1)
    peers=node.get_publishers_info_by_topic(PRIVATE);assert len(peers)==1
    gid=bytes(peers[0].endpoint_gid).hex();assert len(gid)==48
    # Binary STL, exact two double-sided triangles at axial Z=1. Owner-specified
    # hashes/counts and twelve generic links exercise the real shared loader.
    vertices=[((-2,-2,1),(2,-2,1),(2,2,1)),((-2,-2,1),(2,2,1),(-2,2,1))]
    raw=b'\0'*80+struct.pack('<I',2)+b''.join(struct.pack('<12fH',0,0,0,*[v for p in t for v in p],0) for t in vertices)
    asset=tmp_path/'plane.stl';asset.write_bytes(raw);digest=hashlib.sha256(raw).hexdigest()
    params={'enabled':True,'self_exclusion_mode':'dynamic-mesh-v1','base_frame':'base_link',
        'input_topic':INPUT,'output_topic':OUTPUT,'diagnostics_topic':DIAG,
        'max_cloud_age_sec':.3,'tf_timeout_sec':.02,'max_removed_fraction':.1,
        'private_tf_publisher':'/dynamic_filter_test_source','private_tf_gid':gid if request.param=='exact-gid-v1' else '',
        'publisher_identity_mode':request.param,
        'model_mesh_paths':[str(asset)]*12,'model_mesh_sha256':[digest]*12,
        'model_triangle_counts':[2]*12,'model_link_names':['leg'+str(i) for i in range(12)],
        'model_parent_frames':['base_link']*12,'self_tolerance_m':.02}
    # The node is intentionally launched under the OLD private TF remap. Its
    # dedicated global listener must ignore that remap for camera extrinsics.
    args=['--ros-args','-r','/tf:='+PRIVATE,'-r','/tf_static:=/sunhorizon/navigation/belly_self/tf_static']
    import yaml
    for name,value in params.items():args+=['-p',name+':='+yaml.safe_dump(value,default_flow_style=True).strip().removesuffix('\n...').strip()]
    log=(tmp_path/'node.log').open('w+')
    process=subprocess.Popen([os.environ['SELF_EXCLUSION_EXECUTABLE'],*args],stdout=log,stderr=subprocess.STDOUT)
    static=StaticTransformBroadcaster(node)
    camera=TransformStamped();camera.header.frame_id='base_link';camera.child_frame_id='camera_belly_depth_optical_frame';camera.transform.rotation.w=1.;camera.header.stamp=node.get_clock().now().to_msg();static.sendTransform(camera)
    try:
        spin(node,12,lambda:source.get_subscription_count()==1 and tfpub.get_subscription_count()==1 and node.count_publishers(OUTPUT)==1)
        assert process.poll() is None
        assert source.get_subscription_count()==1 and tfpub.get_subscription_count()==1
        spin(node,.3)
        if request.param=='startup-pin-once-v1':
            # Graph presence alone cannot qualify deletion/output before an
            # authenticated callback freezes this candidate's identity.
            before,_=cloud(node.get_clock().now());source.publish(before);spin(node,.1)
            assert not any(m.header==before.header for m in outputs)
        yield node,tfpub,source,outputs,diags,process
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait(timeout=5)
        log.seek(0);print(log.read());log.close()
        node.destroy_subscription(sub);node.destroy_subscription(diagnostic);node.destroy_node();rclpy.shutdown()

def tf_message(stamp, include=12, wrong_parent=False):
    result=TFMessage()
    for i in range(include):
        t=TransformStamped();t.header.frame_id='wrong' if wrong_parent and i==11 else 'base_link'
        t.child_frame_id='leg'+str(i);t.header.stamp=stamp.to_msg();t.transform.rotation.w=1.;result.transforms.append(t)
    return result

def cloud(stamp):
    points=[(0,0,1),(0,0,.9),(0,0,1.1),(0,0,0),(0,0,-1),(float('nan'),0,1),(5,0,1)]+[(5+i,0,1) for i in range(13)]
    msg=PointCloud2();msg.header.frame_id='camera_belly_depth_optical_frame';msg.header.stamp=stamp.to_msg()
    msg.width=10;msg.height=2;msg.point_step=20;msg.row_step=207;msg.is_dense=False
    msg.fields=[PointField(name=n,offset=4*i,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
    msg.fields+=[PointField(name='id',offset=12,datatype=PointField.UINT32,count=1),PointField(name='opaque',offset=16,datatype=PointField.UINT32,count=1)]
    records=[struct.pack('<fffII',*p,i,0xa5000000+i) for i,p in enumerate(points)]
    msg.data=b''.join(records[:10])+b'padding'+b''.join(records[10:])+b'padding'
    return msg,records

def sample(runtime, include=12, concurrent=False, wrong_parent=False):
    node,tfpub,source,outputs,diags,process=runtime
    stamp=node.get_clock().now()-rclpy.duration.Duration(seconds=.015)
    lower=stamp-rclpy.duration.Duration(seconds=.001);upper=stamp+rclpy.duration.Duration(seconds=.001)
    tfpub.publish(tf_message(lower,include,wrong_parent));spin(node,.005)
    msg,records=cloud(stamp)
    if not concurrent:tfpub.publish(tf_message(upper,include,wrong_parent));spin(node,.005)
    source.publish(msg)
    if concurrent:
        # Positive-time lookup must be serviced by a genuinely different TF
        # executor while the cloud callback waits, not just a dedicated flag.
        # The source is already qualified by the caller. Observe the actual
        # fixed-source wait instead of guessing a 5ms executor/DDS schedule.
        def pending():
            return any(r.get('pending_source_ns')==str(stamp.nanoseconds)
                       for r in diagnostic_rows(diags,'dynamic_future_leg_tf_pending'))
        spin(node,.07,pending)
        assert pending(), [(status.message, dict((v.key,v.value) for v in status.values))
                           for report in diags[-3:] for status in report.status]
        tfpub.publish(tf_message(upper,include,wrong_parent))
    spin(node,2,lambda:any(m.header==msg.header for m in outputs))
    assert process.poll() is None
    return msg,records,[m for m in outputs if m.header==msg.header]

def test_private_tf_concurrent_and_record_bytes(runtime):
    # Future-TF holding intentionally starts only after one complete exact-time
    # model has qualified the source. The first frame instead has the bounded
    # initial lookup deadline; test the documented qualified recovery contract.
    assert sample(runtime)[2]
    msg,records,matched=sample(runtime,concurrent=True)
    assert len(matched)==1, [(status.message, dict((v.key,v.value) for v in status.values))
                            for report in runtime[4][-3:] for status in report.status]
    out=matched[0];assert out.header==msg.header and out.fields==msg.fields
    assert out.point_step==20 and out.width==19 and out.height==1 and out.row_step==380
    assert bytes(out.data)==b''.join(records[1:]) # near/far/NaN/zero/nonpositive/miss preserved
    node,tfpub,source,outputs,diags,process=runtime
    spin(node,.25)
    identities=[dict((v.key,v.value) for v in status.values) for msg in diags for status in msg.status]
    assert any(r.get('private_tf_identity_frozen')=='true' and r.get('private_tf_frozen_gid')==bytes(node.get_publishers_info_by_topic(PRIVATE)[0].endpoint_gid).hex() for r in identities)
    # Real private chain missing the twelfth exact-time bracket suppresses the
    # whole frame, then a complete new source frame may recover.
    assert not sample(runtime,include=11)[2]
    assert sample(runtime)[2]

@pytest.mark.parametrize('fault',['wrong_parent','duplicate_owner'])
def test_private_tf_identity_discontinuity_stops_output(runtime,fault):
    node,tfpub,source,outputs,diags,process=runtime
    assert sample(runtime)[2]
    held,_=cloud(node.get_clock().now());source.publish(held);spin(node,.04)
    assert not any(m.header==held.header for m in outputs)  # Qualified future-only wait is now active.
    if fault=='wrong_parent':assert not sample(runtime,wrong_parent=True)[2]
    else:
        duplicate=node.create_publisher(TFMessage,PRIVATE,100)
        spin(node,5,lambda:len(node.get_publishers_info_by_topic(PRIVATE))==2)
        assert len(node.get_publishers_info_by_topic(PRIVATE))==2
        assert not sample(runtime)[2]
        node.destroy_publisher(duplicate);spin(node,.3)
    # Sticky evidence invalidation must not resume using the pre-fault TF cache.
    assert not sample(runtime)[2]

def diagnostic_rows(diags, reason):
    return [dict((v.key,v.value) for v in status.values) for message in diags for status in message.status
            if status.message==reason]

def test_qualified_future_tf_arrival_keeps_original_cloud_and_packed_bytes(runtime):
    node,tfpub,source,outputs,diags,process=runtime
    assert sample(runtime)[2]
    stamp=node.get_clock().now()-rclpy.duration.Duration(seconds=.01)
    tfpub.publish(tf_message(stamp-rclpy.duration.Duration(seconds=.001)));spin(node,.005)
    held,records=cloud(stamp);source.publish(held)
    spin(node,.07,lambda:bool(diagnostic_rows(diags,'dynamic_future_leg_tf_pending')))
    pending=diagnostic_rows(diags,'dynamic_future_leg_tf_pending')[-1]
    original=(pending['pending_source_ns'],pending['pending_first_receipt_ros_ns'],pending['pending_first_receipt_steady_ns'])
    assert int(original[0])==stamp.nanoseconds and not any(m.header==held.header for m in outputs)
    # >20ms late TF can only service the fixed first source, not replace it with
    # this newer cloud. The exact bytes/header remain the measured old input.
    spin(node,.04);new,_=cloud(node.get_clock().now());source.publish(new);spin(node,.02)
    assert not any(m.header==new.header or m.header==held.header for m in outputs)
    tfpub.publish(tf_message(stamp+rclpy.duration.Duration(seconds=.001)))
    spin(node,.2,lambda:any(m.header==held.header for m in outputs))
    matched=[m for m in outputs if m.header==held.header];assert len(matched)==1
    assert bytes(matched[0].data)==b''.join(records[1:]) and matched[0].fields==held.fields
    assert not any(m.header==new.header for m in outputs)
    pending_observations=diagnostic_rows(diags,'dynamic_future_leg_tf_pending')
    assert all((r['pending_source_ns'],r['pending_first_receipt_ros_ns'],r['pending_first_receipt_steady_ns'])==original for r in pending_observations)
    spin(node,.08);assert len([m for m in outputs if m.header==held.header])==1 and process.poll() is None

def test_pending_cloud_is_not_replaced_and_expiry_never_outputs_old_or_dropped_sources(runtime):
    node,tfpub,source,outputs,diags,process=runtime
    assert sample(runtime)[2]
    held,_=cloud(node.get_clock().now());source.publish(held);spin(node,.04)
    assert diagnostic_rows(diags,'dynamic_future_leg_tf_pending')
    original=diagnostic_rows(diags,'dynamic_future_leg_tf_pending')[-1]
    newer=[]
    for _ in range(4):
        message,_=cloud(node.get_clock().now());newer.append(message);source.publish(message);spin(node,.02)
    # Keep the new-input burst well before expiry; later legitimate new anchors
    # must not be confused with inputs discarded while this slot was occupied.
    spin(node,.3,lambda:bool(diagnostic_rows(diags,'dynamic_pending_source_or_receipt_stale')))
    assert diagnostic_rows(diags,'dynamic_pending_source_or_receipt_stale')
    tfpub.publish(tf_message(node.get_clock().now()));spin(node,.06)
    assert not any(m.header==held.header or any(m.header==new.header for new in newer) for m in outputs)
    waits=diagnostic_rows(diags,'dynamic_future_leg_tf_pending')
    assert all(r['pending_source_ns']==original['pending_source_ns'] and
               r['pending_first_receipt_steady_ns']==original['pending_first_receipt_steady_ns'] for r in waits)
    assert sample(runtime)[2] and process.poll() is None  # Only a new, complete source may recover.

@pytest.mark.parametrize('parameters,reason',[
    ({'enabled':'true','self_exclusion_mode':'other'},'unknown_self_exclusion_mode'),
    ({'enabled':'true','self_exclusion_mode':'dynamic-mesh-v1','base_frame':'base_link',
      'input_topic':INPUT,'output_topic':OUTPUT,'publisher_identity_mode':'startup-pin-once-v1',
      'private_tf_publisher':'/dynamic_filter_test_source','private_tf_gid':'"'+('1'*48)+'"'},'private_tf_owner_identity_required'),
    ({'enabled':'true','self_exclusion_mode':'dynamic-mesh-v1'},'dynamic_mesh_requires_exclusive_belly_optical_mode'),
    ({'enabled':'true','self_exclusion_mode':'dynamic-mesh-v1','base_frame':'base_link',
      'input_topic':INPUT,'output_topic':OUTPUT,'additional_exclusion_min_xyz':'[0.0, 0.0, 0.0]',
      'additional_exclusion_max_xyz':'[0.1, 0.1, 0.1]'},'dynamic_mesh_requires_exclusive_belly_optical_mode'),
])
def test_dynamic_mode_rejects_unknown_or_legacy_mix(tmp_path,parameters,reason):
    env=dict(os.environ,ROS_LOCALHOST_ONLY='1',ROS_DOMAIN_ID=str(220+os.getpid()%10))
    argv=[os.environ['SELF_EXCLUSION_EXECUTABLE'],'--ros-args']
    for key,value in parameters.items():argv+=['-p',key+':='+value]
    result=subprocess.run(argv,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,env=env,timeout=12)
    assert result.returncode==1
    assert reason in result.stdout
