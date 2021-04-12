/**
 * @file feature_tracker_node.cc
 * @brief 光流跟踪启动节点
 * @author GWh
 * @version 0.1
 * @date 2021-04-06 16:02:42
 */
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cil-slam/feature_tracker.h>

using namespace std;
using namespace CIL;

FeatureTrackerPtr featureTrackerPtr;

void ImgCallback(const sensor_msgs::ImageConstPtr &img_msg) {
    // cv_bridge::CvImageConstPtr colorImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr grayImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    const cv::Mat &curImg = grayImgPtr->image;

    featureTrackerPtr->Run(curImg);
    featureTrackerPtr->DebugShow();

    Features& currFeatures = featureTrackerPtr->GetFeatures();

    return;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    featureTrackerPtr.reset(new FeatureTracker(n));

    string imageTopic;
    n.param<string>("common/image_topic", imageTopic, string("/front_image_6mm"));
    ros::Subscriber sub_img = n.subscribe(imageTopic, 100, ImgCallback);

    ros::spin();
    return 0;
}
