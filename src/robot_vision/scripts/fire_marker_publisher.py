#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class FireMarkerPublisher:
    def __init__(self):
        rospy.init_node('fire_marker_publisher')
        self.marker_pub = rospy.Publisher('/fire_marker', Marker, queue_size=10)
        rospy.Subscriber("/fire_position_3d", PointStamped, self.callback)
        
    def callback(self, msg):
        marker = Marker()
        marker.header = msg.header  # 继承坐标系和时间戳
        marker.ns = "fire_detection"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.point  # 火焰位置
        
        # 可视化样式设置
        marker.scale.x = 0.3  # 球体直径(m)
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0  # 红色火焰
        marker.color.a = 0.8  # 透明度
        marker.lifetime = rospy.Duration(0.5)  # 持续显示时间
        
        self.marker_pub.publish(marker)

if __name__ == '__main__':
    FireMarkerPublisher()
    rospy.spin()
