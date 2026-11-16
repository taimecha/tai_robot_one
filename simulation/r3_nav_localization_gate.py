#!/usr/bin/env python3
"""Legacy isolated experiment: bounded AMCL motion-onset command interlock.

The production A navigation launch does not run this experiment. It uses the
native Nav2 smoother/collision chain and checks stationary quality at A.

Publishes only the input of the existing velocity smoother. The smoother and
LiDAR collision monitor retain final velocity and braking ownership.
"""
import json
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


ZERO=(0.,0.,0.)


def finite(values):
    return all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)
               for v in values)


def angle(value):return math.atan2(math.sin(value),math.cos(value))


class OnsetGate:
    """No relaxed localization bounds: brake before exhausting the old proof."""
    def __init__(self):
        self.onset=None
        self.session=None

    def filter(self,command,odom,status,now):
        try:
            return self._filter(command,odom,status,now)
        except (ValueError,TypeError,KeyError,IndexError):
            return ZERO,'waiting for valid localization/odometry evidence',{}

    def _filter(self,command,odom,status,now):
        if not finite([now,*command]) or len(command)!=3:
            raise ValueError('invalid command')
        if (not isinstance(status,dict) or status.get('ready') is not True
                or status.get('continuous_localization') is not True
                or status.get('navigation_monitor') is not True
                or not -.05<=now-status['published_ros_s']<=.5):
            return ZERO,'waiting for fresh navigation localization status',{}
        if (not isinstance(odom,dict) or not finite([odom['stamp'],*odom['pose'],*odom['twist']])
                or not -.05<=now-odom['stamp']<=.2):
            return ZERO,'waiting for fresh wheel/IMU odometry',{}
        identity=(status['run_id'],status['map_file'])
        if self.session is None:self.session=identity
        if identity!=self.session:
            return ZERO,'localization session/map changed; restart navigation',{}
        snapshot=status['stationary_snapshot'];source=snapshot['amcl_stamp_s']
        if not finite([source]):raise ValueError('invalid AMCL source')
        if status.get('live_observation_validated') is True and -.05<=now-source<=2.:
            self.onset=None
            return tuple(command),'fresh AMCL navigation',{}
        proof=status.get('motion_prediction')
        predicted=status.get('motion_prediction_validated') is True
        parked=status.get('stationary_cache',{}).get('validated') is True
        if not (predicted or parked):
            return ZERO,'waiting for a fresh AMCL observation',{}
        if self.onset is None:
            if predicted:
                if (not isinstance(proof,dict) or proof.get('validated') is not True
                        or proof.get('ended') is True or proof['source_amcl_stamp_s']!=source
                        or any(not math.isclose(proof[k],v,rel_tol=0.,abs_tol=1e-12)
                               for k,v in (('max_duration_s',2.),('max_distance_m',.08),('max_yaw_rad',.08)))):
                    raise ValueError('invalid onset proof')
                self.onset=dict(source=source,started=proof['started_ros_s'],
                                anchor=list(proof['anchor_odom_xyyaw']))
            elif math.hypot(*odom['twist'][:2])<.005 and abs(odom['twist'][2])<.01:
                if not any(abs(v)>1e-9 for v in command):
                    return ZERO,'parked; no navigation command',{}
                self.onset=dict(source=source,started=now,anchor=list(odom['pose']))
            else:
                return ZERO,'waiting for the bridge motion-onset proof',{}
        onset=self.onset
        if onset['source']!=source:
            return ZERO,'waiting for validation of the changed AMCL source',{}
        anchors=[onset['anchor']]
        if predicted:
            if (not isinstance(proof,dict) or proof.get('ended') is True
                    or proof.get('validated') is not True or proof['source_amcl_stamp_s']!=source):
                raise ValueError('invalid active onset proof')
            anchors.append(proof['anchor_odom_xyyaw'])
        if any(len(p)!=3 or not finite(p) for p in anchors):
            raise ValueError('invalid onset anchor')
        elapsed=now-onset['started']
        distance=max(math.dist(odom['pose'][:2],p[:2]) for p in anchors)
        yaw=max(abs(angle(odom['pose'][2]-p[2])) for p in anchors)
        speed=math.hypot(*odom['twist'][:2]);rotation=abs(odom['twist'][2])
        # Reserve 100 ms for command delivery plus a geometric margin. The
        # angular reserve still assumes a conservative .4 rad/s^2 brake;
        # the smoother uses the original real-drive 1.5 rad/s^2 limit.
        # Keeping the command zero lets that smoother perform the braking.
        latency=.10+max(0.,now-odom['stamp'])
        travel_at_stop=distance+speed*speed/(2*.12)+speed*latency+.003
        yaw_at_stop=yaw+rotation*rotation/(2*.4)+rotation*latency+.005
        evidence=dict(source_amcl_stamp_s=source,elapsed_s=elapsed,
            displacement_m=distance,yaw_change_rad=yaw,
            odom_age_s=now-odom['stamp'],command_and_feedback_reserve_s=latency,
            predicted_stop_distance_m=travel_at_stop,predicted_stop_yaw_rad=yaw_at_stop,
            max_distance_m=.08,max_yaw_rad=.08,max_duration_s=2.)
        if not -.05<=elapsed<1.9 or travel_at_stop>=.08 or yaw_at_stop>=.08:
            return ZERO,'pausing for fresh AMCL before the onset limit',evidence
        return tuple(command),'bounded motion onset with braking reserve',evidence


class NavigationGate(Node):
    def __init__(self):
        super().__init__('r3_navigation_localization_gate')
        self.gate=OnsetGate();self.command=None;self.command_stamp=None
        self.odom=None;self.status=None;self.last_reason=None;self.last_report=0.
        reliable=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.output=self.create_publisher(Twist,'/cmd_vel_localized',1)
        self.report=self.create_publisher(String,'/pallet/r3/navigation_gate/status',reliable)
        self.create_subscription(Twist,'/cmd_vel_nav',self.on_command,1)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
        self.create_subscription(String,'/pallet/r3/localization/status',self.on_status,reliable)
        self.create_timer(.05,self.tick)

    def now(self):return self.get_clock().now().nanoseconds*1e-9
    def on_command(self,message):
        self.command=(message.linear.x,message.linear.y,message.angular.z)
        self.command_stamp=self.now()
    def on_status(self,message):
        try:self.status=json.loads(message.data)
        except (ValueError,TypeError):self.status=None
    def on_odom(self,message):
        if message.header.frame_id!='odom' or message.child_frame_id not in ('base_footprint','base_link'):
            self.odom=None;return
        p=message.pose.pose;q=p.orientation;t=message.twist.twist
        if abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1.)>.02:
            self.odom=None;return
        self.odom=dict(stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9,
            pose=[p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))],
            twist=[t.linear.x,t.linear.y,t.angular.z])
    def tick(self):
        now=self.now();evidence={}
        if self.command_stamp is None or not -.05<=now-self.command_stamp<=.25:
            command,reason=ZERO,'waiting for a fresh Nav2 command'
        else:
            command,reason,evidence=self.gate.filter(self.command,self.odom,self.status,now)
        # Preserve a quiet hardware startup until Nav2 actually sends a
        # command. After the first command, stale/faulted input actively
        # publishes zero so the existing smoother can brake the robot.
        if self.command_stamp is not None:
            message=Twist();message.linear.x,message.linear.y,message.angular.z=command
            self.output.publish(message)
        if reason!=self.last_reason or now-self.last_report>=.5:
            if reason!=self.last_reason:
                self.get_logger().info('Navigation localization gate: '+reason)
            self.report.publish(String(data=json.dumps(dict(published_ros_s=now,reason=reason,
                output=list(command),evidence=evidence),allow_nan=False)))
            self.last_reason=reason;self.last_report=now


def main():
    rclpy.init();node=NavigationGate()
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    except RuntimeError:
        if rclpy.ok():raise
    finally:
        if rclpy.ok() and node.command_stamp is not None:node.output.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
