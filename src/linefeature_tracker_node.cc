/**
 * @file linefeature_tracker_node.cc
 * @brief 线特征提取与跟踪节点
 * @author GWH
 * @version 0.1
 * @date 2021-04-06 20:38:37
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <cil-slam/linefeature_tracker.h>

using namespace std;
using namespace CIL;

LineTrackerPtr lineTrackerPtr;

void ImgCallback(const sensor_msgs::ImageConstPtr &img_msg) {
    // cv_bridge::CvImageConstPtr colorImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr grayImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    const cv::Mat &curImg = grayImgPtr->image;

    lineTrackerPtr->Run(curImg);
    lineTrackerPtr->DebugShow();
    LineFeatures &lineFeatures = lineTrackerPtr->GetFeatures();

    return;
}
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "line_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    lineTrackerPtr.reset(new LineFeatureTracker(n));

    string imageTopic;
    n.param<string>("common/image_topic", imageTopic, string("/front_image_6mm"));
    ros::Subscriber sub_img = n.subscribe(imageTopic, 100, ImgCallback);

    ros::spin();
    return 0;
}
