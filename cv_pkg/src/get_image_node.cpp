#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ros::NodeHandle nh("~");
    try {
        // 将ROS图像转换为OpenCV图像（假设为彩色图像BGR8）
        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 设置保存路径（确保路径存在）
        std::string save_path;
        nh.param<std::string>("save_path", save_path, "default_name");

        // 保存图像
        cv::imwrite(save_path+"captured_image.png", image);

        ROS_INFO("Saved image to %s", save_path.c_str());
        
        // 延时1s
        ros::Duration(1.0).sleep();
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_image_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    ros::spin();
    return 0;
}
