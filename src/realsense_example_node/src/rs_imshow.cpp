
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class ImageConverter {
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

public:
    ImageConverter() : it_(nh_) {
        // 订阅RealSense的RGB图像话题
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1, 
            &ImageConverter::imageCallback, this);
        
        // 创建OpenCV窗口
        cv::namedWindow("RealSense RGB View");
    }

    ~ImageConverter() {
        cv::destroyWindow("RealSense RGB View");
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            // 将ROS图像消息转换为OpenCV图像
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 显示图像
        cv::imshow("RealSense RGB View", cv_ptr->image);
        cv::waitKey(3);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "realsense_rgb_subscriber");
    ImageConverter ic;
    ros::spin();
    return 0;
}
