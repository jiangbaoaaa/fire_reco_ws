
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
        self.fire_pub = rospy.Publisher("/fire_center_px", Point, queue_size=10)
        self.lower_fire = np.array([0, 150, 100])
        self.upper_fire = np.array([30, 255, 255])
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        cv2.namedWindow("Flame Detection", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv2.GaussianBlur(cv_image, (7,7), 0), cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_fire, self.upper_fire)
            
            # 形态学优化
            kernel = np.ones((5,5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 800:
                    x,y,w,h = cv2.boundingRect(cnt)
                    center = Point()
                    center.x = x + w//2
                    center.y = y + h//2
                    self.fire_pub.publish(center)
                    
                    # 可视化标记
                    cv2.rectangle(cv_image, (x,y), (x+w,y+h), (0,0,255), 2)
                    cv2.circle(cv_image, (center.x, center.y), 5, (255,0,0), -1)
                    cv2.putText(cv_image, f"Fire {area}px", (x,y-10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            
            cv2.imshow("Flame Detection", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(str(e))

if __name__ == '__main__':
    rospy.init_node('fire_detector')
    FireDetector()
    rospy.spin()
