/**
 * @file linefeature_tracker.h
 * @brief 线特征提取与跟踪类
 * @author GWH
 * @version 0.1
 * @date 2021-04-06 20:41:18
 */
#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <stdexcept>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/line_descriptor.hpp>

using namespace std;
using cv::line_descriptor::KeyLine;
using cv::line_descriptor::LSDDetector;
using cv::line_descriptor::BinaryDescriptor;
using cv::line_descriptor::BinaryDescriptorMatcher;

namespace CIL {

/**
 * @brief 无符号long long int别名
 */
typedef unsigned long long int FeatureIDType;

/**
 * @brief 一个特征线的结构
 */
struct LineFeatureMetaData {
    FeatureIDType id;                  // 特征线Id
    int lifeTime;                      // 特征线被跟踪的次数
    cv::Point2f startPoint;            // 特征线的起点像素坐标（去除畸变前的）
    cv::Point2f endPoint;              // 特征线的终点像素坐标（去除畸变前的）
    cv::Point2f startPointRectified;   // 特征线的起点像素坐标（去除畸变后的）
    cv::Point2f endPointRectified;     // 特征线的终点像素坐标（去除畸变后的）

    cv::Vec3d n;                       // 直线的plucker坐标
    cv::Vec3d v;                       // 直线的plucker坐标
};

/**
 * @brief 一帧图像中的特征线
 */
typedef vector<LineFeatureMetaData> LineFeatures;

class LineFeatureTracker {
   public:
    /**
     * @brief 构造函数，随后需要LoadParameters加载参数
     */
    LineFeatureTracker();

    /**
     * @brief 构造函数2，省去手动加载参数的过程
     */
    LineFeatureTracker(ros::NodeHandle& nh);

    /**
     * @brief 析构函数
     */
   ~LineFeatureTracker(){}

    /**
     * @brief 删除复制构造和operator=函数
     */
    LineFeatureTracker(const LineFeatureTracker&) = delete;
    LineFeatureTracker operator=(const LineFeatureTracker&) = delete;

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
    LineFeatures& GetFeatures();

    /**
     * @brief OpenCv可视化结果
     */
    cv::Mat DebugShow();

    typedef shared_ptr<LineFeatureTracker> Ptr;
    typedef shared_ptr<const LineFeatureTracker> ConstPtr;

   private:
    /**
     * @brief 光流跟踪以及相机相关参数
     */
    struct ProcessorConfig {
        int imgCols;                     // 图像的长度
        int imgRows;                     // 图像的宽度
        int equalize;                    // 非0则进行直方图分布均衡化

        int maxCnt;                      // 图像中线段最大数量
        int octaves;                     // 图像金字塔的层数
        float scale;                     // 图像金字塔的缩放比例
        float goodMatchesDistance;       // lbd最大匹配距离
        int minDist;                     // 平行直线的最小间隔
        int minLineLength;               // 提取出来的线段的最短长度

        float lineLinkDistance;          // 直线拼接时的最长距离阈值（像素）
        float lineLinkAngle;             // 直线平行的最大弧度阈值（弧度）
        float lineMergeDistance;         // 直线合并时的最大距离阈值（像素）
 
        cv::Vec4d camIntrinsics;         // 相机的内参矩阵fx,fy,cx,cy
        string camDistortionModel;       // 相机的畸变模型
        cv::Vec4d camDistortionCoeffs;   // 相机的畸变系数k1,k2,p1,p2
    };

    /**
     * @brief 初始化第一帧
     */
    void InitializeFirstFrame();

    /**
     * @brief 前后帧之间的直线匹配
     */
    void Matching();

    /**
     * @brief 检测新的线段并计算描述子
     */
    void DetectNewLine();

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

    bool isFirstImg_;                                 // 是否为首帧图像
    FeatureIDType nextFeatureId_;                     // 下一个特征线的Id（用来为每一个特征赋予ID）
    cv::Mat undistMap1_;                              // 图像去畸变x轴映射
    cv::Mat undistMap2_;                              // 图像去畸变y轴映射

    
    cv::Mat prevImg_;                                 // 上一帧图像
    cv::Mat currImg_;                                 // 当前帧图像
    LineFeatures currLines_;                          // 当前图像中的特征线（包含特征点的全部信息）
    vector<KeyLine> prevCvLines_;                     // 上一帧图像中的特征线（作为运算的中间变量）
    cv::Mat prevLinesDescriptor_;                        // 上一帧图像中特征线的描述子
    vector<KeyLine> currCvLines_;                     // 当前图像中的特征线（作为运算的中间变量）
    cv::Mat currLinesDescriptor_;                        // 当前帧图像中特征线的描述子
    vector<KeyLine> newAddLines_;                     // 新提取的特征线
    cv::Mat newAddLinesDescriptor_;                   // 新提取的线段的描述子

    cv::Ptr<LSDDetector> lsd_;                        // lsd线段检测器
    cv::Ptr<BinaryDescriptor> lbd_;                   // lbd描述子计算器
    cv::Ptr<BinaryDescriptorMatcher> lineMatcher_;    // 线描述子匹配器
};

typedef LineFeatureTracker::Ptr LineTrackerPtr;
typedef LineFeatureTracker::ConstPtr LineTrackerConstPtr;

}  // end namespace CIL
