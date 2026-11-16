#!/usr/bin/env python3
"""Stationary inspection of camera obstacles after measured floor alignment.

The original RGB/depth calibration and cloud are unchanged. A robust observed
floor is rotated onto z=0 in base_footprint after ten seconds of camera warmup
and two seconds of stable fits. A frozen transform is never adapted.
This diagnostic correction does not establish absolute depth-range accuracy.
No TF, navigation goals or motor commands are published.
"""
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


INPUT_TOPIC='/camera/depth/points_nav_corrected'
OUTPUT_TOPIC='/camera/depth/points_nav_floor_corrected'
STATUS_TOPIC='/camera/depth/nav_floor_alignment/status'
FRAME='base_footprint'


def quaternion_matrix(quaternion):
    q=np.asarray(quaternion,dtype=float)
    if q.shape!=(4,) or not np.all(np.isfinite(q)) or abs(np.dot(q,q)-1.)>.02:
        raise ValueError('invalid camera transform quaternion')
    x,y,z,w=q/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def floor_transform(normal,offset_m,pivot=(0.,0.,0.)):
    """Level n.p+d=0 while preserving the physical camera origin's XY.

    R maps n onto +Z; t_z=d maps the measured floor to z=0. Choosing the XY
    translation about the camera pivot avoids inventing a camera XY shift.
    """
    n=np.asarray(normal,dtype=float)
    if (n.shape!=(3,) or not np.all(np.isfinite(n)) or np.linalg.norm(n)<.9
            or not math.isfinite(offset_m)):
        raise ValueError('invalid floor plane')
    n=n/np.linalg.norm(n)
    if n[2]<=0:
        raise ValueError('floor normal must point up')
    cross=np.cross(n,np.array([0.,0.,1.]))
    sine=np.linalg.norm(cross)
    if sine<1e-12:
        rotation=np.eye(3)
    else:
        x,y,z=cross
        skew=np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])
        rotation=np.eye(3)+skew+skew@skew*((1.-n[2])/(sine*sine))
    pivot=np.asarray(pivot,dtype=float)
    if pivot.shape!=(3,) or not np.all(np.isfinite(pivot)):
        raise ValueError('invalid camera-origin pivot')
    translation=pivot-rotation@pivot
    translation[2]=float(offset_m)
    return rotation,translation


def normal_angle_deg(a,b):
    a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
    return math.degrees(math.acos(float(np.clip(np.dot(a,b)/(
        np.linalg.norm(a)*np.linalg.norm(b)),-1.,1.))))


def fit_floor(points, *, max_points=2500, trials=64, min_inliers=300, seed=7):
    """Bounded RANSAC on expected ground; return explicit acceptance evidence."""
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3:
        raise ValueError('floor points must be an Nx3 array')
    roi=(np.all(np.isfinite(points),axis=1)&(points[:,0]>=.55)&(points[:,0]<=3.)
         &(np.abs(points[:,1])<=1.2)&(points[:,2]>=-.20)&(points[:,2]<=.10))
    candidates=points[roi]
    available=len(candidates)
    if available>max_points:
        candidates=candidates[np.linspace(0,available-1,max_points,dtype=int)]
    count=len(candidates)
    result=dict(accepted=False,candidate_count=available,sampled_count=count,
                ground_band=dict(x_m=[.55,3.],abs_y_m=1.2,z_m=[-.20,.10]),
                max_tilt_deg=6.,max_vertical_offset_m=.1,
                inlier_threshold_m=.012)
    if count<min_inliers:
        return dict(result,reason='insufficient expected-ground points')
    rng=np.random.default_rng(seed)
    best=None;best_count=0
    for _ in range(trials):
        a,b,c=candidates[rng.choice(count,3,replace=False)]
        normal=np.cross(b-a,c-a)
        length=np.linalg.norm(normal)
        if length<1e-6:
            continue
        normal/=length
        if normal[2]<0:
            normal=-normal
        offset=-float(normal@a)
        if normal[2]<math.cos(math.radians(6.)) or abs(offset)>.1:
            continue
        mask=np.abs(candidates@normal+offset)<=.012
        mass=int(np.count_nonzero(mask))
        if mass>best_count:
            best_count=mass;best=mask
    if best is None or best_count<min_inliers:
        return dict(result,reason='no dominant plausible floor within tilt/offset limits')
    # Two least-squares refinements use only robust inliers, not object tops.
    for _ in range(2):
        selected=candidates[best]
        centre=np.mean(selected,axis=0)
        _,vectors=np.linalg.eigh((selected-centre).T@(selected-centre)/len(selected))
        normal=vectors[:,0]
        if normal[2]<0:
            normal=-normal
        offset=-float(normal@centre)
        best=np.abs(candidates@normal+offset)<=.012
    selected=candidates[best]
    residual=np.abs(selected@normal+offset)
    inliers=len(selected)
    tilt=normal_angle_deg(normal,[0.,0.,1.])
    span=np.ptp(selected[:,:2],axis=0) if inliers else np.zeros(2)
    ratio=inliers/count
    median=float(np.median(residual)) if inliers else None
    p90=float(np.quantile(residual,.9)) if inliers else None
    accepted=(inliers>=min_inliers and ratio>=.5 and span[0]>=1. and span[1]>=.5
              and tilt<=6. and abs(offset)<=.1 and median<=.006 and p90<=.012)
    return dict(result,accepted=accepted,normal=normal.tolist(),offset_m=offset,
                tilt_deg=tilt,inlier_count=inliers,inlier_ratio=ratio,
                median_residual_m=median,p90_residual_m=p90,
                support_x_span_m=float(span[0]),support_y_span_m=float(span[1]),
                reason=('dominant floor fit accepted' if accepted else
                        'floor fit lacks dominance/spatial support or exceeds geometry/residual limits'))


class FloorAlignmentLock:
    """Require stable distinct floor observations; never adapt a frozen plane."""
    def __init__(self):
        self.samples=deque(maxlen=12)
        self.changed_samples=deque(maxlen=12)
        self.recovery_samples=deque(maxlen=12)
        self.recovering=False
        self.first_stamp=None
        self.frozen=None
        self.fault=None
        self.last_stamp=None
        self.latest_fit=None
        self.ready=False
        self.reason='waiting for stationary floor observations'

    def withhold(self,reason):
        self.ready=False;self.samples.clear();self.changed_samples.clear();self.recovery_samples.clear()
        self.recovering=self.frozen is not None
        self.reason=self.fault or reason

    def motion(self):
        self.withhold('robot is moving; stationary inspection only')
        if self.frozen is not None:
            self.fault='robot moved after floor alignment was frozen; restart stationary inspection'
            self.reason=self.fault

    @staticmethod
    def _append_window(samples,stamp,normal,offset,seconds):
        if samples and stamp-samples[-1][0]>.9:
            samples.clear()
        samples.append((stamp,np.asarray(normal,dtype=float),offset))
        while len(samples)>2 and stamp-samples[0][0]>seconds+.6:
            samples.popleft()

    def observe(self,fit,stamp,stationary=True,pivot=(0.,0.,0.)):
        self.ready=False
        if self.fault:
            self.reason=self.fault;return False
        if not stationary:
            self.motion();return False
        if not math.isfinite(stamp):
            self.withhold('invalid floor cloud timestamp');return False
        if self.last_stamp is not None and stamp<=self.last_stamp:
            self.reason='waiting for a distinct floor cloud';return False
        if self.last_stamp is not None and stamp-self.last_stamp>.9:
            self.withhold('floor observations interrupted; stable verification required')
        self.last_stamp=stamp;self.latest_fit=fit
        if self.first_stamp is None:
            self.first_stamp=stamp
        if stamp-self.first_stamp<10.:
            self.samples.clear()
            self.reason=f'camera floor warmup: {stamp-self.first_stamp:.1f}/10.0 seconds';return False
        pivot_array=np.asarray(pivot,dtype=float)
        if pivot_array.shape!=(3,) or not np.all(np.isfinite(pivot_array)):
            self.withhold('camera pivot transform is invalid');return False
        if (self.frozen is not None and
                np.linalg.norm(pivot_array-self.frozen['camera_origin_base_xyz'])>.005):
            self.fault='camera pivot changed after alignment was frozen; restart inspection and check camera mounting'
            self.reason=self.fault;return False
        if not fit.get('accepted',False):
            self.withhold(fit.get('reason','floor fit rejected'));return False
        normal=fit['normal'];offset=fit['offset_m']
        if self.frozen is not None:
            changed=(normal_angle_deg(normal,self.frozen['normal'])>.5
                     or abs(offset-self.frozen['offset_m'])>.015)
            if changed:
                self.recovering=True;self.recovery_samples.clear()
                self._append_window(self.changed_samples,stamp,normal,offset,1.)
                first=self.changed_samples[0]
                if any(normal_angle_deg(n,first[1])>.3 or abs(d-first[2])>.01
                       for _,n,d in self.changed_samples):
                    self.changed_samples.clear()
                    self.changed_samples.append((stamp,np.asarray(normal,dtype=float),offset))
                duration=stamp-self.changed_samples[0][0]
                if len(self.changed_samples)>=3 and duration>=1.:
                    self.fault='floor changed consistently after alignment was frozen; restart inspection and check camera mounting'
                self.reason=self.fault or 'floor fit disagrees with frozen plane; camera obstacles withheld'
                return False
            self.changed_samples.clear()
            if self.recovering:
                self._append_window(self.recovery_samples,stamp,normal,offset,2.)
                duration=stamp-self.recovery_samples[0][0]
                if len(self.recovery_samples)<5 or duration<2.:
                    self.reason=f'verifying original frozen floor after interruption: {duration:.1f}/2.0 seconds'
                    return False
                self.recovering=False;self.recovery_samples.clear()
            self.ready=True;self.reason='frozen floor alignment verified; absolute depth range unverified';return True
        self._append_window(self.samples,stamp,normal,offset,2.)
        first=self.samples[0]
        if any(normal_angle_deg(n,first[1])>.3 or abs(d-first[2])>.01
               for _,n,d in self.samples):
            self.samples.clear();self.samples.append((stamp,np.asarray(normal),offset))
            self.reason='floor estimate is changing; stable observations required';return False
        duration=stamp-self.samples[0][0]
        if len(self.samples)<5 or duration<2.:
            self.reason=f'collecting stable floor fits: {duration:.1f}/2.0 seconds';return False
        normal=np.median([n for _,n,_ in self.samples],axis=0)
        normal/=np.linalg.norm(normal)
        offset=float(np.median([d for _,_,d in self.samples]))
        rotation,translation=floor_transform(normal,offset,pivot)
        self.frozen=dict(normal=normal.tolist(),offset_m=offset,
                         rotation=rotation.tolist(),translation_m=translation.tolist(),
                         camera_origin_base_xyz=np.asarray(pivot,dtype=float).tolist(),
                         frozen_ros_s=stamp,stable_duration_s=duration,
                         fit_count=len(self.samples),warmup_duration_s=10.,
                         aggregation='component median normal, renormalized; median offset')
        self.ready=True;self.reason='floor alignment frozen; absolute depth range unverified'
        return True


def cloud_xyz(message):
    """Read XYZ with declared offsets and row strides, including padded rows."""
    fields={field.name:field for field in message.fields}
    if message.width<=0 or message.height<=0 or any(name not in fields for name in ('x','y','z')):
        raise ValueError('camera cloud has no XYZ points')
    byte_order='>' if message.is_bigendian else '<'
    formats=[];offsets=[]
    for name in ('x','y','z'):
        field=fields[name]
        if field.count!=1 or field.datatype not in (PointField.FLOAT32,PointField.FLOAT64):
            raise ValueError('camera XYZ fields must be scalar floats')
        formats.append(byte_order+('f4' if field.datatype==PointField.FLOAT32 else 'f8'))
        offsets.append(field.offset)
    if message.row_step<message.width*message.point_step or len(message.data)<message.row_step*message.height:
        raise ValueError('camera cloud data/strides are truncated')
    dtype=np.dtype(dict(names=['x','y','z'],formats=formats,offsets=offsets,itemsize=message.point_step))
    array=np.ndarray((message.height,message.width),dtype=dtype,buffer=message.data,
                     strides=(message.row_step,message.point_step))
    return np.stack([array[name] for name in ('x','y','z')],axis=-1).reshape(-1,3)


class FloorAlignedCloud(Node):
    def __init__(self):
        super().__init__('r3_camera_floor_alignment')
        self.input_topic=str(self.declare_parameter('input_topic',INPUT_TOPIC).value)
        self.output_topic=str(self.declare_parameter('output_topic',OUTPUT_TOPIC).value)
        self.status_topic=str(self.declare_parameter('status_topic',STATUS_TOPIC).value)
        self.camera_frame=str(self.declare_parameter('camera_frame','camera_color_optical_frame').value)
        report=str(self.declare_parameter('report_file','').value)
        self.report_file=Path(report) if report else None
        if (not self.input_topic or not self.output_topic or not self.status_topic or not self.camera_frame
                or self.report_file is not None and not self.report_file.is_absolute()):
            raise ValueError('topics must be nonempty; report_file must be empty or absolute')
        self.lock=FloorAlignmentLock()
        self.odom=None;self.stationary_since=None
        self.last_cloud_stamp=None;self.last_fit_wall=0.;self.last_publish_wall=0.
        self.issue='waiting for fresh camera cloud and stationary odom'
        self.tf=Buffer(cache_time=Duration(seconds=5.))
        self.listener=TransformListener(self.tf,self)
        self.output=self.create_publisher(PointCloud2,self.output_topic,qos_profile_sensor_data)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status=self.create_publisher(String,self.status_topic,qos)
        self.create_subscription(PointCloud2,self.input_topic,self.on_cloud,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_timer(.5,self.publish_status)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_odom(self,message):
        twist=message.twist.twist
        valid=(message.header.frame_id=='odom' and message.child_frame_id in ('base_footprint','base_link') and
               all(math.isfinite(v) for v in (twist.linear.x,twist.linear.y,twist.angular.z)))
        moving=valid and (math.hypot(twist.linear.x,twist.linear.y)>=.005 or abs(twist.angular.z)>=.01)
        still=valid and not moving
        self.odom=message
        if not still:
            self.stationary_since=None
            if moving:
                self.lock.motion()
            else:
                self.lock.withhold('waiting for valid stationary odom')
        elif self.stationary_since is None:
            self.stationary_since=time.monotonic()

    def stationary(self):
        return (self.odom is not None and -.05<=self.now()-self.stamp(self.odom)<=.25
                and self.stationary_since is not None
                and time.monotonic()-self.stationary_since>=.5)

    def on_cloud(self,message):
        wall=time.monotonic();stamp=self.stamp(message)
        if not -.05<=self.now()-stamp<=.5:
            self.issue='camera cloud is stale';self.lock.withhold(self.issue);return
        self.last_cloud_stamp=stamp
        if not self.stationary():
            self.issue='waiting for fresh odom and stationary robot';self.lock.withhold(self.issue);return
        if wall-self.last_publish_wall<.25:
            return
        try:
            points=cloud_xyz(message)
            points=points[np.all(np.isfinite(points),axis=1)]
            if len(points)>50000:
                points=points[np.linspace(0,len(points)-1,50000,dtype=int)]
            if message.header.frame_id!=FRAME:
                # This is the rigid camera mounting transform, not odom/map
                # motion. Astra broadcasts some fixed optical links on /tf;
                # requesting the cloud stamp can be milliseconds ahead of
                # that broadcast and starve otherwise valid floor samples.
                transform=self.tf.lookup_transform(FRAME,message.header.frame_id,
                    Time(),timeout=Duration(seconds=.03)).transform
                q=transform.rotation;t=transform.translation
                if not all(math.isfinite(v) for v in (t.x,t.y,t.z)):
                    raise ValueError('invalid camera transform translation')
                rotation=quaternion_matrix([q.x,q.y,q.z,q.w])
                points=points@rotation.T+np.array([t.x,t.y,t.z])
                camera_origin=[t.x,t.y,t.z]
            else:
                # A pre-transformed input still needs the physical sensor
                # pivot so leveling cannot invent a horizontal extrinsic.
                transform=self.tf.lookup_transform(FRAME,self.camera_frame,
                    Time(),timeout=Duration(seconds=.03)).transform
                t=transform.translation
                camera_origin=[t.x,t.y,t.z]
            if wall-self.last_fit_wall>=.5:
                self.lock.observe(fit_floor(points),stamp,pivot=camera_origin)
                self.last_fit_wall=wall
            self.issue=self.lock.reason
            if not self.lock.ready:
                return
            frozen=self.lock.frozen
            corrected=points@np.asarray(frozen['rotation']).T+np.asarray(frozen['translation_m'])
            output=PointCloud2()
            output.header.stamp=message.header.stamp;output.header.frame_id=FRAME
            output.height=1;output.width=len(corrected)
            output.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)
                           for i,name in enumerate(('x','y','z'))]
            output.is_bigendian=False;output.is_dense=True
            output.point_step=12;output.row_step=12*len(corrected)
            output.data=np.asarray(corrected,dtype='<f4').tobytes()
            self.output.publish(output);self.last_publish_wall=wall
        except (ValueError,TransformException) as exc:
            self.issue='floor correction unavailable: '+str(exc);self.lock.withhold(self.issue)

    def publish_status(self):
        fresh=self.last_cloud_stamp is not None and -.05<=self.now()-self.last_cloud_stamp<=.75
        stationary=self.stationary()
        ready=self.lock.ready and stationary and fresh
        reason=('waiting for fresh camera cloud' if not fresh else
                'waiting for fresh odom and stationary robot' if not stationary else self.issue)
        value=dict(ready=ready,reason=reason,published_ros_s=self.now(),frame_id=FRAME,
            input_topic=self.input_topic,output_topic=self.output_topic,
            stationary_inspection_only=True,robot_stationary=stationary,
            alignment_frozen=self.lock.frozen is not None,
            alignment_fault=self.lock.fault,reverifying_original_plane=self.lock.recovering,
            frozen_alignment=self.lock.frozen,latest_floor_fit=self.lock.latest_fit,
            absolute_range_calibration_verified=False,
            original_calibration_modified=False)
        serialized=json.dumps(value,allow_nan=False)
        self.status.publish(String(data=serialized))
        if self.report_file is not None:
            try:
                self.report_file.parent.mkdir(parents=True,exist_ok=True)
                temporary=self.report_file.with_suffix(self.report_file.suffix+'.tmp')
                temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n')
                temporary.replace(self.report_file)
            except OSError as exc:
                self.get_logger().warning('Cannot save floor inspection report: '+str(exc),throttle_duration_sec=5.)


def main():
    rclpy.init();node=FloorAlignedCloud()
    node.get_logger().info('Stationary camera floor inspection; no motion commands; absolute depth range unverified')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
