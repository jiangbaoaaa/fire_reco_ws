
#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

class FireDetector:
    def __init__(self):
        self.bridge = CvBridge()
        # 动态HSV阈值（初始值）
        self.fire_pub = rospy.Publisher("/fire_center_px", Point, queue_size=10)
        self.lower_fire = np.array([0, 150, 100])  
        self.upper_fire = np.array([30, 255, 255])
        
        # 运动检测相关
        self.prev_gray = None
        self.fgbg = cv2.createBackgroundSubtractorMOG2(history=500, varThreshold=16)
        
        # 订阅图像
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", 
                                        Image, self.image_callback)
        
    def dynamic_threshold(self, hsv_img):
        """动态调整饱和度阈值"""
        avg_sat = np.mean(hsv_img[:,:,1])
        self.lower_fire[1] = max(100, int(avg_sat * 0.65))
        
    def motion_analysis(self, frame):
        """运动特征分析"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        fgmask = self.fgbg.apply(gray)
        return cv2.medianBlur(fgmask, 5)
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 预处理
            blurred = cv2.GaussianBlur(cv_image, (7,7), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
            
            # 动态阈值调整
            self.dynamic_threshold(hsv)
            
            # 颜色检测
            color_mask = cv2.inRange(hsv, self.lower_fire, self.upper_fire)
            color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, 
                                        np.ones((7,7), np.uint8))
            
            # 运动检测
            motion_mask = self.motion_analysis(cv_image)
            
            # 多特征融合
            combined_mask = cv2.bitwise_and(color_mask, motion_mask)
            
            # 精确轮廓检测
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL,
                                         cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 800:  # 增大面积阈值减少误检
                    x,y,w,h = cv2.boundingRect(cnt)
                    #计算火焰中心点
                    center = Point()
                    center.x = x + w//2
                    center.y = y + h//2
                    self.fire_pub.publish(center)

                    aspect_ratio = float(w)/h
                    if 0.5 < aspect_ratio < 2.0:  # 火焰通常不是极端长条形
                        
                        #可视化标记点
                        cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,0,255), 2)     #红色框标记火焰
                        cv2.circle(cv_image, (center.x, center.y), 5, (255,0,0), -1)   #蓝色小点表示火焰中心位置
                        cv2.putText(cv_image, f'Fire {area:.0f}', (x,y-10),         
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)     #绿色表示像素面积
            
            # 显示调试视图
            debug_view = np.hstack([
                cv_image,
                cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)
            ])
            cv2.imshow("Fire Detection", debug_view)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(str(e))

if __name__ == '__main__':
    rospy.init_node('fire_detector')
    detector = FireDetector()
    rospy.spin()
