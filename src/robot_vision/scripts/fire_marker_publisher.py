#!/usr/bin/env python
import rospy
import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class FireMarkerPublisher:
    def __init__(self):
        rospy.init_node('fire_marker_publisher')
        self.marker_pub = rospy.Publisher('/fire_marker', Marker, queue_size=10)
        rospy.Subscriber("/fire_position_3d", PointStamped, self.callback)
        
        # 动态更新参数
        self.last_position = None
        self.current_marker = Marker()
        self.update_threshold = 0.3  # 位置变化阈值(米)
        self.stable_timeout = rospy.Duration(2.0)  # 稳定持续时间
        
        # 初始化默认Marker
        self.current_marker.ns = "fire_detection"
        self.current_marker.id = 0
        self.current_marker.type = Marker.SPHERE
        self.current_marker.action = Marker.ADD
        self.current_marker.scale.x = self.current_marker.scale.y = self.current_marker.scale.z = 0.1
        self.current_marker.color.r = 1.0
        self.current_marker.color.a = 0.8
        self.current_marker.lifetime = rospy.Duration(0)

    def callback(self, msg):
        current_pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        
        # 首次检测或超过稳定时间
        if self.last_position is None or \
           (rospy.Time.now() - self.current_marker.header.stamp) > self.stable_timeout:
            self._update_marker(msg, current_pos)
            return
            
        # 计算位置变化量
        displacement = np.linalg.norm(current_pos - self.last_position)
        
        # 超过阈值则更新
        if displacement > self.update_threshold:
            self._update_marker(msg, current_pos)

    def _update_marker(self, msg, new_pos):
        self.last_position = new_pos
        self.current_marker.header = msg.header
        self.current_marker.pose.position = msg.point
        #self.current_marker.header.stamp = rospy.Time.now()  # 重置时间戳
        self.marker_pub.publish(self.current_marker)

if __name__ == '__main__':
    FireMarkerPublisher()
    rospy.spin()
