/**
 * @file feature_tracker.h
 * @brief 光流跟踪类
 * @author GWH
 * @version 0.1
 * @date 2021-04-06 16:01:20
 */
#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <stdexcept>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

using namespace std;

namespace CIL {

/**
 * @brief 无符号long long int别名
 */
typedef unsigned long long int FeatureIDType;

/**
 * @brief 一个特征点的结构
 */
struct FeatureMetaData {
    FeatureIDType id;               // 特征点Id
    int lifeTime;                   // 特征点被跟踪的次数
    cv::Point2f ptInImg;            // 特征点的像素坐标（去除畸变前的）
    cv::Point2f ptInImgRectified;   // 特征点的像素坐标（去除畸变后的）
    cv::Point3f ptInNormalizedCam;  // 特征点在归一化成像平面坐标（去除畸变后的）
};

/**
 * @brief 一帧图像中的特征点
 */
typedef vector<FeatureMetaData> Features;

class FeatureTracker {
   public:
    /**
     * @brief 构造函数，随后需要LoadParameters加载参数
     */
    FeatureTracker();

    /**
     * @brief 构造函数2，省去手动加载参数的过程
     */
    FeatureTracker(ros::NodeHandle& nh);

    /**
     * @brief 析构函数
     */
   ~FeatureTracker(){}

    /**
     * @brief 删除复制构造和operator=函数
     */
    FeatureTracker(const FeatureTracker&) = delete;
    FeatureTracker operator=(const FeatureTracker&) = delete;

    /**
     * @brief 加载参数至内置结构体ProcessorConfig
     */
    bool LoadParameters(const ros::NodeHandle& nh);

    /**
     * @brief  执行光流跟踪 
     *
     * @param img 灰度图像
     */
    bool Run(const cv::Mat& img);

    /**
     * @brief 获取光流跟踪的特征点
     */
    Features& GetFeatures();

    /**
     * @brief OpenCv可视化结果
     */
    cv::Mat DebugShow();

    typedef shared_ptr<FeatureTracker> Ptr;
    typedef shared_ptr<const FeatureTracker> ConstPtr;

   private:
    /**
     * @brief 光流跟踪以及相机相关参数
     */
    struct ProcessorConfig {
        int imgCols;                     // 图像的长度
        int imgRows;                     // 图像的宽度
        int maxCnt;                      // 光流跟踪中最大特征点数量
        int minDist;                     // 特征点间最小间隔
        int equalize;                    // 非0则进行直方图分布均衡化
        double ransacThreshold;          // 前后帧Ransac去除外点的阈值

        cv::Vec4d camIntrinsics;         // 相机的内参矩阵fx,fy,cx,cy
        string camDistortionModel;       // 相机的畸变模型
        cv::Vec4d camDistortionCoeffs;   // 相机的畸变系数k1,k2,p1,p2
    };

    /**
     * @brief 初始化第一帧
     */
    void InitializeFirstFrame();

    /**
     * @brief 前后帧之间的光流跟踪
     */
    void TrackFeatures();

    /**
     * @brief 使用Ransac算法计算基础矩阵来进一步去除外点
     */
    void RejectWithF();

    /**
     * @brief 设置蒙版，将已经检测出特征点的区域给掩盖掉, 便于之后其他区域用于检测新的特征点
     */
    void SetMask();

    /**
     * @brief 对pt_in进行畸变去除 
     *
     * @param pt_in                 输入单点
     * @param cam_intrinsics        相机内参（fx,fy,cx,cy）
     * @param cam_distortion_coeffs 相机的畸变系数（k1,k2,p1,p2）
     * @param cam_distortion_model  相机的畸变模型（"radtan"等）
     * @param pt_out                去除畸变后的pt_in像素坐标
     * @param pt_normp_out          去除畸变后的pt_in归一化平面坐标
     */
    void UndistortPoint(const cv::Point2f& pt_in, 
                        const cv::Vec4d& cam_intrinsics, 
                        const cv::Vec4d& cam_distortion_coeffs, 
                        const string& cam_distortion_model, 
                        cv::Point2f& pt_out, 
                        cv::Point3f& pt_normp_out);

    /**
     * @brief 对pts_in进行畸变去除 
     *
     * @param pts_in                输入点集
     * @param cam_intrinsics        相机内参（fx,fy,cx,cy）
     * @param cam_distortion_coeffs 相机的畸变系数（k1,k2,p1,p2）
     * @param cam_distortion_model  相机的畸变模型（"radtan"等）
     * @param pt_out                去除畸变后的pts_in像素坐标
     * @param pt_normp_out          去除畸变后的pts_in归一化平面坐标
     */
    void UndistortPoints(const vector<cv::Point2f>& pts_in, 
                         const cv::Vec4d& cam_intrinsics, 
                         const cv::Vec4d& cam_distortion_coeffs, 
                         const string& cam_distortion_model, 
                         vector<cv::Point2f>& pts_out, 
                         vector<cv::Point3f>& pts_normp_out);

    /**
     * @brief 检测像素点pt是否落在图像平面内（在内返回true）
     */
    bool InBorder(const cv::Point2f& pt);

    /**
     * @brief 去除raw_vec中在markers对应位置为0的元素
     */
    template <typename T>
    void RemoveUnmarkedElements(vector<T>& raw_vec, const vector<uchar>& markers) {
        if (raw_vec.size() != markers.size()) {
            throw logic_error("The input size of raw_vec and markers does not match");
        }
        int j = 0;
        for (int i = 0; i < static_cast<int>(raw_vec.size()); i++)
            if (markers[i]) raw_vec[j++] = raw_vec[i];
        raw_vec.resize(j);
        return;
    }

    ProcessorConfig config_;
    ros::NodeHandle nh_;

    bool isFirstImg_;                    // 是否为首帧图像
    FeatureIDType nextFeatureId_;        // 下一个特征点的Id（用来为每一个特征赋予ID）

    cv::Mat mask_;                       // 使特征点均匀分布的蒙版
    cv::Mat prevImg_;                    // 上一帧图像
    cv::Mat currImg_;                    // 当前帧图像
    Features currFeatures_;              // 当前图像中的特征点（包含特征点的全部信息）
    vector<cv::Point2f> prevCvFeatures_; // 上一帧图像中的特征点（作为运算的中间变量）
    vector<cv::Point2f> currCvFeatures_; // 当前图像中的特征点（作为运算的中间变量）
    vector<cv::Point2f> newAddPts_;      // 新提取的特征点
        
};

typedef FeatureTracker::Ptr FeatureTrackerPtr;
typedef FeatureTracker::ConstPtr FeatureTrackerConstPtr;

}  // end namespace CIL
