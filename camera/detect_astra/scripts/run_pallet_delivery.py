#!/usr/bin/env python3
"""Operator commands for saved pallet-1 delivery and stationary unloading."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Empty, Float64MultiArray
from tf2_ros import Buffer, TransformListener
import yaml
from pallet_process_guard import arm_cycle_parent

ROOT=Path(__file__).resolve().parents[1]


def nav2_endpoint_key(infos):
    if (len(infos)!=1 or infos[0].node_name!='bt_navigator'
            or infos[0].node_namespace!='/'):
        return None
    return bytes(infos[0].endpoint_gid)


def wait_for_unique_nav2(node,client,pump,timeout=10.,settle_s=1.):
    """Wait for discovery to settle; never authorize zero/duplicate servers."""
    deadline=time.monotonic()+timeout
    stable_since=None;previous=None;observed=[];ready=False
    while time.monotonic()<deadline:
        pump(.05)
        infos=node.get_publishers_info_by_topic('/navigate_to_pose/_action/status')
        key=nav2_endpoint_key(infos)
        ready=client.server_is_ready()
        observed=[p.node_namespace.rstrip('/')+'/'+p.node_name for p in infos]
        if key is None or not ready:
            stable_since=None;previous=None
            continue
        now=time.monotonic()
        if key!=previous:
            stable_since=now;previous=key
        if now-stable_since>=settle_s:
            return key
    raise RuntimeError('Chưa xác nhận duy nhất một máy chủ Nav2 sau '
                       f'{timeout:g} giây: status_publishers={observed}, action_ready={ready}')


def angle(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def at_station(pose,goal):
    return (math.hypot(pose[0]-goal[0],pose[1]-goal[1])<=.08
            and abs(math.atan2(math.sin(pose[2]-goal[2]),math.cos(pose[2]-goal[2])))<=math.radians(5))


def recognition_backoff_goal(goal,distance):
    if not math.isfinite(distance) or not 0<=distance<=.6:
        raise ValueError('Điểm nhận diện dự phòng phải cách goal cũ từ 0 tới 0,6 m')
    return (goal[0]-distance*math.cos(goal[2]),goal[1]-distance*math.sin(goal[2]),goal[2])


def station_navigation_ready(station, name, revalidate_recognition=False):
    # A recognition station may be revisited explicitly for fresh visual
    # validation. This never authorizes a pending pickup/dropoff or changes
    # the stored station's validation status.
    return (station.get('status') == 'recorded' or
            (name == 'recognition' and revalidate_recognition and
             station.get('status') == 'needs_visual_revalidation'))


def stationary_report(samples,now):
    recent=[p for p in samples if p[0]>=now-1.]
    span=recent[-1][0]-recent[0][0] if recent else 0.
    v=max((abs(p[1]) for p in recent),default=0.)
    w=max((abs(p[2]) for p in recent),default=0.)
    finite=all(math.isfinite(value) for p in recent for value in p[1:])
    return dict(stationary=bool(recent and finite and span>=.8 and v<=.003 and w<=.015),
                window_s=round(span,3),samples=len(recent),max_linear_mps=v,max_angular_rps=w)


def wait_for_stationary(sensors,pose,report,active,pump,timeout=8.):
    """Requalify after discovery/parameter callbacks; never bypass stillness."""
    end=time.monotonic()+timeout;last={};last_print=time.monotonic()
    while time.monotonic()<end:
        pump(.05)
        if active():raise RuntimeError('Nav2 đang có goal khác; chưa gửi lệnh mới')
        lf=sensors();current=pose();last=report()
        last.update(lift_state=lf.get('state'),lift_velocity=lf.get('velocity'))
        if last['stationary'] and lf.get('state')=='IDLE' and abs(lf['velocity'])<=.002:
            return lf,current
        if time.monotonic()-last_print>=2.:
            print('Chờ xác nhận xe/càng đứng yên: '+json.dumps(last,ensure_ascii=False),flush=True)
            last_print=time.monotonic()
    raise RuntimeError('Chưa xác nhận xe/càng đứng yên sau '+str(timeout)+' s: '+
                       json.dumps(last,ensure_ascii=False))


def lower_issue(pose,goal,active,still,lift):
    if active:return 'Nav2 đang chạy; chưa hạ pallet'
    if not at_station(pose,goal):return 'Chưa tới đúng goal giao hàng; chưa hạ pallet'
    if not still:return 'Xe chưa đứng yên liên tục 0,8 giây'
    if (lift.get('homed') is not True or lift.get('fault')!='NONE'
            or lift.get('serial_connected') is not True):return 'Càng chưa sẵn sàng hoặc có lỗi'
    return ''


def main():
    arm_cycle_parent()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=('go','return','lower','stop'))
    parser.add_argument('--recognition-backoff',type=float,default=0.,
                        help='Điểm dự phòng xa pallet hơn goal nhận diện, tối đa 0,6 m; không ghi đè goal')
    parser.add_argument('--revalidate-recognition',action='store_true',
                        help='Cho phép quay về goal nhận diện đã lưu đang cần kiểm chứng lại bằng camera')
    args=parser.parse_args()
    returning=args.operation=='return'
    if args.revalidate_recognition and not returning:
        parser.error('--revalidate-recognition chỉ dùng với return')
    goal=None
    if args.operation!='stop':
        name='recognition' if returning else 'dropoff'
        station=yaml.safe_load((ROOT.parents[1]/'ros2_ws/config/pallet_goals.yaml').read_text())['pallets']['pallet_1'][name]
        if not station_navigation_ready(station,name,args.revalidate_recognition):
            parser.error(f'Goal pallet 1 {name} chưa được lưu mới')
        goal=(station['position']['x'],station['position']['y'],station['yaw_rad'])
    if args.recognition_backoff:
        if not returning:parser.error('Điểm dự phòng chỉ dùng khi càng trống quay về nhận diện')
        goal=recognition_backoff_goal(goal,args.recognition_backoff)
        print(f'Điểm dự phòng xa hơn goal nhận diện {args.recognition_backoff*100:g} cm; goal lưu giữ nguyên.',flush=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n=rclpy.create_node('pallet_delivery_operator');states={};quiet=deque()
    buf=Buffer();listener=TransformListener(buf,n)
    def receive(key,msg):states[key]=(msg,time.monotonic())
    def odom(msg):
        receive('odom',msg);wall=time.monotonic()
        quiet.append((wall,msg.twist.twist.linear.x,msg.twist.twist.angular.z))
        # Keep a TIME window, independent of sensor rate or callback bursts.
        while quiet and quiet[0][0]<wall-1.2:quiet.popleft()
    n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    n.create_subscription(LaserScan,'/scan_filtered',lambda m:receive('laser',m),qos_profile_sensor_data)
    n.create_subscription(String,'/lift/status',lambda m:receive('lift',m),1)
    n.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',lambda m:receive('nav',m),
                          QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    stop_lift=n.create_publisher(Empty,'/lift/stop',1)
    stop_base=n.create_publisher(Twist,'/cmd_vel_nav',1)
    lift_pub=n.create_publisher(Float64MultiArray,'/lift_controller/commands',1) if args.operation=='lower' else None
    cancel=n.create_client(CancelGoal,'/navigate_to_pose/_action/cancel_goal')
    client=ActionClient(n,NavigateToPose,'/navigate_to_pose');handle=None;commanded=False;success=False
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.03)
    def active():
        return any(s.status in (1,2,3) for s in states.get('nav',(GoalStatusArray(),0))[0].status_list)
    def lift():
        data=json.loads(states['lift'][0].data)
        if (data.get('serial_connected') is not True or data.get('homed') is not True
                or data.get('fault')!='NONE' or not math.isfinite(data['position'])
                or not math.isfinite(data['velocity'])):raise RuntimeError('Phản hồi càng không hợp lệ')
        return data
    def sensors():
        wall=time.monotonic();now=n.get_clock().now().nanoseconds/1e9
        for key in ('odom','laser','lift'):
            if key not in states or wall-states[key][1]>.5:raise RuntimeError('Mất dữ liệu mới: '+key)
        for key in ('odom','laser'):
            stamp=states[key][0].header.stamp
            if not -.05<=now-stamp.sec-stamp.nanosec*1e-9<=.5:raise RuntimeError('Thời gian dữ liệu quá hạn: '+key)
        return lift()
    def pose():
        t=buf.lookup_transform('map','base_footprint',rclpy.time.Time())
        age=n.get_clock().now().nanoseconds/1e9-t.header.stamp.sec-t.header.stamp.nanosec*1e-9
        if not -.5<=age<=1.5:raise RuntimeError('Định vị bản đồ quá hạn')
        return (t.transform.translation.x,t.transform.translation.y,angle(t.transform.rotation))
    def still():
        return stationary_report(quiet,time.monotonic())['stationary']
    def stationary():
        return wait_for_stationary(sensors,pose,lambda:stationary_report(quiet,time.monotonic()),active,pump)
    def halt():
        if cancel.wait_for_service(timeout_sec=1):
            f=cancel.call_async(CancelGoal.Request())
            rclpy.spin_until_future_complete(n,f,timeout_sec=2)
        end=time.monotonic()+1.
        while time.monotonic()<end:
            stop_base.publish(Twist());stop_lift.publish(Empty());pump(.08)
    try:
        if args.operation=='stop':halt();success=True;return 0
        end=time.monotonic()+10
        while time.monotonic()<end:
            pump(.1)
            if all(k in states for k in ('odom','laser','lift')) and buf.can_transform('map','base_footprint',rclpy.time.Time()) and still():break
        lifecycle=n.create_client(GetState,'/bt_navigator/get_state')
        if not lifecycle.wait_for_service(timeout_sec=3):raise RuntimeError('Nav2 chưa sẵn sàng')
        f=lifecycle.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if not f.done() or f.result().current_state.id!=3:raise RuntimeError('Nav2 chưa active')
        # A newly started action server has no latched status sample until
        # its first goal. Verify the single live server, then let any retained
        # active-goal status arrive before deciding whether it is idle.
        nav_owner=wait_for_unique_nav2(n,client,pump)
        pump(1.)
        lf,current=stationary()
        if args.operation in ('go','return'):
            if returning and lf['position']>.01:raise RuntimeError('Càng phải trống và về cao độ thấp trước khi quay về nhận diện')
            if not returning and lf['position']<.05:raise RuntimeError('Càng chưa ở cao độ mang pallet')
            footprint=('[[0.78,0.10],[0.78,-0.10],[0.38,-0.25],[-0.33,-0.25],[-0.33,0.25],[0.38,0.25]]'
                if returning else '[[0.80, 0.25], [0.80, -0.25], [-0.33, -0.25], [-0.33, 0.25]]')
            for target,params in [
                    ('/local_costmap/local_costmap',[Parameter('footprint',value=footprint)]),
                    ('/global_costmap/global_costmap',[Parameter('footprint',value=footprint)]),
                    ('/controller_server',[Parameter('FollowPath.desired_linear_vel',value=.12),
                                            Parameter('FollowPath.rotate_to_heading_angular_vel',value=.20),
                                            # Recognition is a visual staging station. Use 7 cm,
                                            # within the existing 8-cm arrival verification,
                                            # so skid during final yaw does not chase a 5-cm disk.
                                            Parameter('general_goal_checker.xy_goal_tolerance',value=.07 if returning else .05)]),
                    ('/behavior_server',[Parameter('max_rotational_vel',value=.20)])]:
                parameters=AsyncParameterClient(n,target)
                if not parameters.wait_for_services(timeout_sec=3):raise RuntimeError('Thiếu server: '+target)
                future=parameters.set_parameters(params)
                rclpy.spin_until_future_complete(n,future,timeout_sec=3)
                if not future.done() or not all(r.successful for r in future.result().results):
                    raise RuntimeError('Chưa nạp được footprint/tốc độ chở pallet: '+target)
            pump(.5)
            latest,_=stationary()
            if abs(latest['position']-lf['position'])>.005:
                raise RuntimeError('Cao độ càng thay đổi trong lúc chuẩn bị; chưa gửi goal')
            if (returning and latest['position']>.01) or (not returning and latest['position']<.05):
                raise RuntimeError('Cao độ càng không còn phù hợp trước khi gửi goal')
            lf=latest
            if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Nav2 chưa sẵn sàng')
            if nav2_endpoint_key(n.get_publishers_info_by_topic('/navigate_to_pose/_action/status'))!=nav_owner:
                raise RuntimeError('Máy chủ Nav2 thay đổi trong lúc chuẩn bị; chưa gửi goal')
            request=NavigateToPose.Goal();request.pose.header.frame_id='map';request.pose.header.stamp=n.get_clock().now().to_msg()
            request.pose.pose.position.x,request.pose.pose.position.y=goal[:2]
            request.pose.pose.orientation.z=math.sin(goal[2]/2);request.pose.pose.orientation.w=math.cos(goal[2]/2)
            feedback={}
            def update(msg):feedback.update(path_remaining_m=round(msg.feedback.distance_remaining,3),recoveries=msg.feedback.number_of_recoveries)
            commanded=True;f=client.send_goal_async(request,feedback_callback=update)
            rclpy.spin_until_future_complete(n,f,timeout_sec=8)
            if not f.done():raise RuntimeError('Chưa xác nhận nhận goal; gửi dừng')
            handle=f.result()
            if not handle.accepted:commanded=False;raise RuntimeError('Nav2 từ chối goal')
            result=handle.get_result_async();start=time.monotonic();last_print=0.;height=lf['position']
            label='nhận diện' if returning else 'giao'
            print(f'Đang tới goal {label} pallet 1: {goal}. Ctrl+C để hủy và dừng.',flush=True)
            while not result.done() and time.monotonic()-start<180:
                pump(.05);lf=sensors();current=pose()
                if abs(lf['position']-height)>.005 or lf['state']!='IDLE':raise RuntimeError('Cao độ càng thay đổi khi di chuyển')
                if time.monotonic()-last_print>2:
                    progress={**feedback,'distance_to_goal_m':round(math.hypot(current[0]-goal[0],current[1]-goal[1]),3)}
                    if progress.get('path_remaining_m')==0. and progress['distance_to_goal_m']>.08:
                        progress['path_remaining_m']=None
                    print(json.dumps(progress),flush=True);last_print=time.monotonic()
            if not result.done():raise RuntimeError('Quá thời gian; hủy goal')
            response=result.result()
            if response.status!=4:
                if response.result.error_code==208:
                    raise RuntimeError('Nav2 không tìm được đường hợp lệ tới goal (208). Xe đã dừng; chưa hạ pallet.')
                raise RuntimeError('Nav2 chưa tới goal: '+str(response.result))
            end=time.monotonic()+5
            while not still() and time.monotonic()<end:pump(.1)
            if not still() or not at_station(pose(),goal):raise RuntimeError('Chưa xác nhận xe đứng yên trong vùng goal')
            success=True;print('ARRIVED: đã tới goal nhận diện, xe đứng yên.' if returning else
                'ARRIVED: đã tới goal, xe đứng yên. Chạy lệnh lower riêng để đặt pallet xuống.',flush=True)
        else:
            issue=lower_issue(current,goal,active(),still(),lf)
            if issue:raise RuntimeError(issue)
            if not 0<=lf['position']<=.20:raise RuntimeError('Cao độ càng ngoài phạm vi hạ pallet')
            anchor=current;settled=None;start=time.monotonic();last_print=0.
            other=[p for p in n.get_publishers_info_by_topic('/lift_controller/commands') if p.node_name!=n.get_name()]
            if other:raise RuntimeError('Có nguồn điều khiển càng khác')
            commanded=True
            initial_height=lf['position']
            while time.monotonic()-start<25:
                pump(.08);lf=sensors();current=pose()
                issue=lower_issue(current,goal,active(),still(),lf)
                if issue:raise RuntimeError(issue)
                if math.dist(current[:2],anchor[:2])>.01:raise RuntimeError('Xe đã dịch trong lúc hạ')
                if lf['position']>initial_height+.004:raise RuntimeError('Càng đi sai hướng hạ')
                stop_base.publish(Twist());lift_pub.publish(Float64MultiArray(data=[0.]))
                ready=lf['position']<=.002 and abs(lf['velocity'])<.002 and lf['state']=='IDLE'
                settled=(settled if settled is not None else time.monotonic()) if ready else None
                if settled is not None and time.monotonic()-settled>=.8:
                    stop_lift.publish(Empty());success=True
                    print('LOWERED: càng về 0 cm, xe đứng yên. Chưa rút càng.',flush=True);break
                if time.monotonic()-last_print>1:
                    print(f'Đang hạ: {lf["position"]*100:.1f} cm',flush=True);last_print=time.monotonic()
            if not success:raise RuntimeError('Hạ càng quá thời gian; đã dừng')
        return 0
    except KeyboardInterrupt:print('Đang dừng xe và càng…',flush=True);return 1
    except Exception as exc:print(str(exc),flush=True);return 1
    finally:
        if commanded and not success:halt()
        n.destroy_node();rclpy.shutdown()


if __name__=='__main__':raise SystemExit(main())
