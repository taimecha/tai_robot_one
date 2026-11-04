#!/usr/bin/env python3
"""Operator terminal: preview / insert / insert-and-lift / lift / stop / reset."""
import argparse
import json
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy
from pallet_pickup_admission import SensorWindow
from pallet_process_guard import arm_cycle_parent
from pallet_laser_timing import LASER_TIMEOUT_S


def insertion_readiness(states, wall, recover=False):
    """Wait on authoritative live gates; the server still validates the plan."""
    for name in ('handling','alignment','recognition'):
        state,received=states.get(name,({},0.))
        if not state or not 0<=wall-received<=1.:
            return False,'Chờ trạng thái mới: '+name
    handling=states['handling'][0];alignment=states['alignment'][0]
    recognition=states['recognition'][0]
    if (handling.get('state') not in (('IDLE','BLOCKED','STOPPED','LOWERED') if recover else ('IDLE',))
            or handling.get('active') is not False):
        raise RuntimeError('Bộ đưa càng không ở IDLE; không tự tiếp tục bước cũ')
    if ((not recover and alignment.get('state') not in ('ALIGNED','CHECKPOINT_READY')) or alignment.get('active') is not False
            or alignment.get('preparing') is True):
        raise RuntimeError('Cần căn chỉnh đạt ALIGNED trước; chưa gửi lệnh đưa càng')
    if recover and handling.get('frozen_resume_ready') is True:
        if recognition.get('enabled') is False and recognition.get('busy') is False:return True,''
        return False,'Chờ YOLO tắt/xử lý xong trước khi tiếp tục quỹ đạo đã chốt'
    if (recognition.get('phase')!='TRACKING' or recognition.get('locked') is not True
            or recognition.get('enabled') is not False or recognition.get('busy') is not False):
        return False,'Chờ xác nhận pallet và YOLO tắt/xử lý xong'
    health=recognition.get('rgbd_health') or {}
    if (recognition.get('measurement_ready') is not True or health.get('stable') is not True):
        return False,'Chờ cửa sổ RGB-D ổn định 5 giây; xe chưa nhận lệnh đưa càng'
    return True,''


def main():
    arm_cycle_parent()
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=('preview','insert','lift','stop','reset'))
    parser.add_argument('--json',action='store_true',help='In đầy đủ quỹ đạo ở lệnh preview')
    parser.add_argument('--ready-timeout',type=float,default=15.,
                        help='Chờ dữ liệu đạt trước khi đưa càng, tối đa 30 giây (mặc định 15)')
    args=parser.parse_args()
    if not 0<args.ready_timeout<=30:parser.error('--ready-timeout phải lớn hơn 0 và không quá 30')
    # Keep ROS alive while Python handles Ctrl+C and sends the explicit STOP.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n=rclpy.create_node('pallet_pickup_operator')
    status={};received=0.;run_id=None;requested=False;finished=False
    recover=args.operation.startswith('resume-')
    insertion=args.operation in ('insert','insert-and-lift','resume-insert','resume-insert-and-lift')
    moving=insertion or args.operation in ('lift','lower')
    readiness_states={}
    sensors={'odometry':SensorWindow(.20),'laser':SensorWindow(LASER_TIMEOUT_S)}
    def sensor_callback(name):
        def receive(msg):
            sensors[name].observe(msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                time.monotonic(),n.get_clock().now().nanoseconds/1e9)
        return receive
    def readiness_callback(name):
        def receive(msg):
            try:
                data=json.loads(msg.data)
                if not isinstance(data,dict):raise ValueError('Invalid status')
                readiness_states[name]=(data,time.monotonic())
            except (ValueError,TypeError):readiness_states.pop(name,None)
        return receive
    def on_status(msg):
        nonlocal status,received
        try:
            status=json.loads(msg.data);received=time.monotonic()
            readiness_states['handling']=(status,received)
        except ValueError:status={}
    sub=n.create_subscription(String,'/pallet/handling/status',on_status,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if insertion:
        sensor_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        n.create_subscription(Odometry,'/odom',sensor_callback('odometry'),sensor_qos)
        n.create_subscription(LaserScan,'/scan_filtered',sensor_callback('laser'),sensor_qos)
        n.create_subscription(String,'/pallet/alignment/status',readiness_callback('alignment'),1)
        n.create_subscription(String,'/pallet/recognition/status',readiness_callback('recognition'),
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    heartbeat=n.create_publisher(String,'/pallet/handling/heartbeat',1) if moving else None
    stop=n.create_client(Trigger,'/pallet/handling/stop')
    operation=('resume' if recover else 'start') if insertion else args.operation.replace('-','_')
    typ=SetBool if insertion else Trigger
    client=n.create_client(typ,'/pallet/handling/'+operation)
    code=1
    try:
        if not client.wait_for_service(timeout_sec=5):raise RuntimeError('Chưa mở stack handle_pallet mới')
        req=typ.Request()
        if insertion:req.data=args.operation.endswith('and-lift')
        if insertion:
            print(f'Chờ dữ liệu đạt trước khi đưa càng (tối đa {args.ready_timeout:g} giây); '
                  'Ctrl+C để hủy. Chưa gửi lệnh chuyển động.',flush=True)
            end=time.monotonic()+args.ready_timeout
            stable_since=None;last_wait_print=0.;reason='Chưa có dữ liệu mới'
            while time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.02)
                now=time.monotonic()
                ready,reason=insertion_readiness(readiness_states,now,recover)
                if ready:
                    ros=n.get_clock().now().nanoseconds/1e9
                    for name,window in sensors.items():
                        if not window.ready(now,ros):
                            ready=False;reason='Chờ '+name+' mới/liên tục 5 giây; chưa gửi lệnh đưa càng';break
                # Let discovery/startup settle through several live heartbeats,
                # then send ONE request. Never retry an accepted/rejected start.
                stable_since=(stable_since if stable_since is not None else now) if ready else None
                if stable_since is not None and now-stable_since>=.4:break
                if not ready and now-last_wait_print>=1.:
                    print(reason,flush=True);last_wait_print=now
            else:raise RuntimeError('Hết thời gian chờ; chưa gửi lệnh đưa càng. '+reason)
        requested=moving
        future=client.call_async(req);end=time.monotonic()+8
        while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        if not future.done():raise RuntimeError('Dịch vụ quá hạn; gửi dừng, không tự thử lại')
        result=future.result()
        if not result.success:
            requested=False
            raise RuntimeError(result.message)
        if not moving:
            if args.operation in ('preview','resume-preview') and not args.json:
                data=json.loads(result.message);plan=data['plan']
                print(f"Quỹ đạo odom: {plan['travel_m']:.3f} m; goal {plan['goal']}",flush=True)
                print(f"Cao độ càng lúc vào: {plan['entry_height_m']*100:.1f} cm từ HOME",flush=True)
                print("Xem trước đoạn tới checkpoint; đường đưa càng cuối chỉ chốt sau phép đo mới. Không dùng goal pickup bản đồ.",flush=True)
                print('Chỉ xem trước, chưa gửi lệnh chuyển động.',flush=True)
            else:print(result.message,flush=True)
            finished=True;return 0
        run_id=result.message
        print('Đã nhận lệnh. Giữ terminal này mở; Ctrl+C để dừng xe và càng.',flush=True)
        started=time.monotonic();last_heartbeat=last_print=started-1
        while time.monotonic()-started<150:
            now=time.monotonic()
            if now-last_heartbeat>=.09:
                heartbeat.publish(String(data=run_id));last_heartbeat=now
            rclpy.spin_once(n,timeout_sec=.02)
            if now-started>2 and (not status or now-received>1.):raise RuntimeError('Mất trạng thái bộ đưa càng')
            if status.get('run_id')!=run_id:continue
            if now-last_print>=.5:
                lf=status.get('lift') or {}
                print(f"{status['state']}: {status['reason']} | càng={lf.get('position')} m",flush=True)
                last_print=now
            if not status['active']:
                finished=True
                if status['state'] in ('INSERTED','COMPLETE','LOWERED'):code=0
                else:print('Đã dừng: '+status['reason'],flush=True)
                break
        else:raise RuntimeError('Terminal hết thời gian; gửi dừng')
    except KeyboardInterrupt:
        print('\nĐang dừng xe và càng…' if requested else
              '\nĐã hủy; chưa gửi lệnh chuyển động.',flush=True)
    except Exception as exc:print(str(exc),flush=True)
    finally:
        if requested and not finished:
            if stop.wait_for_service(timeout_sec=1):
                future=stop.call_async(Trigger.Request());end=time.monotonic()+2
                while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
                if future.done():print(future.result().message,flush=True)
            # If the service is unreachable, heartbeat expiry closes the gate;
            # the ESP32 SET lease independently expires in 350 ms.
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return code


if __name__=='__main__':raise SystemExit(main())
