#!/usr/bin/env python3
"""Read-only physical footprint and recognition goal for RViz inspection."""
import math
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from pallet_staging_safety import FOOTPRINT

ROOT=Path(__file__).resolve().parents[1]


class ObstacleView(Node):
    def __init__(self):
        super().__init__('pallet_obstacle_view')
        self.publisher=self.create_publisher(MarkerArray,'/pallet_view/markers',1)
        self.goal=yaml.safe_load((ROOT.parents[1]/'ros2_ws/config/pallet_goals.yaml').read_text())['pallets']['pallet_1']['recognition']
        self.create_timer(.2,self.publish)

    def marker(self,name,frame,color):
        m=Marker();m.header.frame_id=frame;m.ns=name;m.id=0;m.action=Marker.ADD
        m.pose.orientation.w=1.;m.color.r,m.color.g,m.color.b=color;m.color.a=1.
        m.frame_locked=True
        return m

    def polygon(self,name,vertices,color,pose=(0.,0.,0.),frame='base_footprint'):
        m=self.marker(name,frame,color);m.type=Marker.LINE_STRIP;m.scale.x=.012
        x,y,a=pose;c,s=math.cos(a),math.sin(a)
        m.points=[Point(x=x+c*u-s*v,y=y+s*u+c*v,z=.03) for u,v in (*vertices,vertices[0])]
        return m

    def publish(self):
        markers=[self.polygon('physical_body_and_forks',FOOTPRINT,(0.,1.,0.))]
        # Explicit deletes also clear overlays already cached in an open RViz.
        for name in ('live_staging_stop_zone','last_rejected_stop_zone',
                     'last_blocking_lidar_cell','recorded_rejection_label'):
            deleted=self.marker(name,'base_footprint',(0.,0.,0.))
            deleted.action=Marker.DELETE;markers.append(deleted)
        m=self.marker('saved_recognition_goal','map',(0.,.5,1.));m.type=Marker.ARROW
        m.pose.position.x=self.goal['position']['x'];m.pose.position.y=self.goal['position']['y']
        m.pose.orientation.z=self.goal['orientation']['z'];m.pose.orientation.w=self.goal['orientation']['w']
        m.scale.x=.35;m.scale.y=.045;m.scale.z=.06;markers.append(m)
        self.publisher.publish(MarkerArray(markers=markers))


def main():
    rclpy.init();node=ObstacleView()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
