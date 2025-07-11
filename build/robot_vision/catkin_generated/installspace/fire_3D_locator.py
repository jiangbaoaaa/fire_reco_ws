#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge
import numpy as np

class Fire3DLocator:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.intrinsics = None
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.info_callback)
        rospy.Subscriber("/fire_center_px", Point, self.fire_callback)
        self.pub = rospy.Publisher("/fire_position_3d", PointStamped, queue_size=10)
        cv2.namedWindow("Depth Visualization", cv2.WINDOW_NORMAL)

    def info_callback(self, msg):
        if not self.intrinsics:
            self.intrinsics = {
                'fx': msg.K[0], 'fy': msg.K[4],
                'cx': msg.K[2], 'cy': msg.K[5]
            }

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        # 深度图归一化显示
        norm_depth = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cv2.imshow("Depth Visualization", cv2.applyColorMap(norm_depth, cv2.COLORMAP_JET))
        cv2.waitKey(1)

    def fire_callback(self, msg):
        if self.depth_image is None or self.intrinsics is None:
            return
        
        u, v = int(msg.x), int(msg.y)
        depth = self.depth_image[v, u] / 1000.0  # mm转m


         # 计算相机坐标系下的3D坐标
        if 0.1 < depth <10.0:
            point_3d = PointStamped()
            point_3d.header.frame_id = "camera_depth_optical_frame"  # 使用相机光学坐标系:ml-citation{ref="3" data="citationList"}
            point_3d.header.stamp = rospy.Time.now()
        
            # 像素坐标转3D坐标（针孔相机模型）:ml-citation{ref="1" data="citationList"}
            point_3d.point.x = (u - self.intrinsics['cx']) * depth / self.intrinsics['fx']
            point_3d.point.y = (v - self.intrinsics['cy']) * depth / self.intrinsics['fy']
            point_3d.point.z = depth
        
            self.pub.publish(point_3d)  # 发布3D坐标

            #终端显示火焰位置
            rospy.loginfo("\n火焰位置(相机坐标系):\n"
                        f"X: {point_3d.point.x:.3f} m\n"
                        f"Y: {point_3d.point.y:.3f} m\n"
                        f"Z: {point_3d.point.z:.3f} m\n"
                        f"像素坐标: ({u}, {v})")
            

if __name__ == '__main__':
    rospy.init_node('fire_3d_locator')
    Fire3DLocator()
    rospy.spin()
