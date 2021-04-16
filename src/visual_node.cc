/**
 * @file visual_node.cc
 * @brief 点线特征跟踪节点
 * @author GWH
 * @version 0.1
 * @date 2021-04-07 20:54:52
 */
#include "ros/console_backend.h"
#include <iostream>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <cil-slam/feature_tracker.h>
#include <cil-slam/linefeature_tracker.h>
#include <cil-slam/timer.h>
#include <cil_slam/FeatureLines.h>
#include <cil_slam/FeaturePoints.h>
#include <cil_slam/ImgInfo.h>

using namespace std;
using namespace CIL;

/* 点线特征跟踪器 */
FeatureTrackerPtr featureTrackerPtr;
LineTrackerPtr lineTrackerPtr;

/* 消息缓存buffer */
queue<cil_slam::FeatureLines> lineFeatureBuff_;
queue<cil_slam::FeaturePoints> pointFeatureBuff_;
queue<sensor_msgs::ImageConstPtr> imgBuff_;

/* 线程时间同步 */
std::mutex m_buff_;
std::condition_variable con_;

/* 发布视觉特征（图像、点特征、线特征） */
ros::Publisher visualFeaturesPub_;

void LineImgCallback(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr colorImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr grayImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    const cv::Mat &curImg = grayImgPtr->image;

    lineTrackerPtr->Run(curImg);
    // lineTrackerPtr->DebugShow();
    LineFeatures &lineFeatures = lineTrackerPtr->GetFeatures();

    cil_slam::FeatureLines rosLineFeatures;
    rosLineFeatures.header = img_msg->header;
    for (auto& it : lineFeatures) {
        rosLineFeatures.id.push_back(it.id);
        rosLineFeatures.startPointX.push_back(it.startPointRectified.x);
        rosLineFeatures.startPointY.push_back(it.startPointRectified.y);
        rosLineFeatures.endPointX.push_back(it.endPointRectified.x);
        rosLineFeatures.endPointY.push_back(it.endPointRectified.y);
    }

    m_buff_.lock();
    lineFeatureBuff_.push(rosLineFeatures);
    imgBuff_.push(img_msg);
    m_buff_.unlock();
    con_.notify_one();

    return;
}

void PointImgCallback(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr colorImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::BGR8);
    cv_bridge::CvImageConstPtr grayImgPtr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    const cv::Mat &curImg = grayImgPtr->image;

    featureTrackerPtr->Run(curImg);
    // featureTrackerPtr->DebugShow();
    Features& currFeatures = featureTrackerPtr->GetFeatures();

    cil_slam::FeaturePoints rosPointFeatures;
    rosPointFeatures.header = img_msg->header;
    for (auto& it : currFeatures) {
        rosPointFeatures.id.push_back(it.id);
        rosPointFeatures.cnt.push_back(it.lifeTime);
        rosPointFeatures.u.push_back(it.ptInImgRectified.x);
        rosPointFeatures.v.push_back(it.ptInImgRectified.y);
        rosPointFeatures.x.push_back(it.ptInNormalizedCam.x);
        rosPointFeatures.y.push_back(it.ptInNormalizedCam.y);
        rosPointFeatures.z.push_back(it.ptInNormalizedCam.z);
    }

    m_buff_.lock();
    pointFeatureBuff_.push(rosPointFeatures);
    m_buff_.unlock();
    con_.notify_one();

    return;
}

cil_slam::ImgInfo GetMeasurements() {
    cil_slam::ImgInfo imgInfo;
    if (pointFeatureBuff_.empty() || lineFeatureBuff_.empty()) {
        return imgInfo;
    }
    if (pointFeatureBuff_.front().header.stamp.toNSec() == lineFeatureBuff_.front().header.stamp.toNSec()) {
        imgInfo.header = pointFeatureBuff_.front().header;
        imgInfo.rawImg = *(imgBuff_.front());
        imgInfo.featurePoints = pointFeatureBuff_.front();
        imgInfo.featureLines = lineFeatureBuff_.front();

        imgBuff_.pop();
        pointFeatureBuff_.pop();
        lineFeatureBuff_.pop();
    }

    return imgInfo;
}

void Process() {
    while(true) {
        Timer timer;
        timer.start();
        unique_lock<std::mutex> lk(m_buff_);
        cil_slam::ImgInfo imgInfo;
        con_.wait(lk, [&]{
                imgInfo = GetMeasurements();
                return imgInfo.featureLines.id.size();
                });
        lk.unlock();

        if(visualFeaturesPub_.getNumSubscribers()) {
            visualFeaturesPub_.publish(imgInfo);
        }
        ROS_DEBUG("Visual feature tracker costs: %f ms", timer.stop());
    }

    return;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "visual__tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    featureTrackerPtr.reset(new FeatureTracker(n));
    lineTrackerPtr.reset(new LineFeatureTracker(n));

    string imageTopic;
    n.param<string>("common/image_topic", imageTopic, string("/front_image_6mm"));

    // 使用两个线程
    ros::AsyncSpinner spinner(2);

    ros::Subscriber sub_img_1 = n.subscribe(imageTopic, 1000, PointImgCallback, ros::TransportHints().tcpNoDelay());   
    ros::Subscriber sub_img_2 = n.subscribe(imageTopic, 1000, LineImgCallback, ros::TransportHints().tcpNoDelay());   

    visualFeaturesPub_ = n.advertise<cil_slam::ImgInfo>("/visual_feature", 100);

    std::thread filter_process{Process};

    spinner.start();
    ros::waitForShutdown();
    return 0;
}
