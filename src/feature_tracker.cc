/**
 * @file feature_tracker.cc
 * @brief 光流跟踪的实现（参考msckf_vio以及vins_mono）
 * @author GWH
 * @version 0.1
 * @date 2021-04-06 16:01:40
 */
#include <cil-slam/feature_tracker.h>
#include <cil-slam/timer.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace CIL {

FeatureTracker::FeatureTracker() : isFirstImg_(true), nextFeatureId_(0) { 
    ROS_INFO("LK feature tracker begin ==============================");
    return; 
}

FeatureTracker::FeatureTracker(ros::NodeHandle& nh) : nh_(nh), isFirstImg_(true), nextFeatureId_(0) {
    ROS_INFO("LK feature tracker begin ==============================");
    LoadParameters(nh_);
    return;
}

bool FeatureTracker::LoadParameters(const ros::NodeHandle& nh) {
    //1. camera calibration parameters from config/config.yaml
    vector<int> camResolutionTemp(2);
    nh.getParam("cam/resolution", camResolutionTemp);
    config_.imgCols = camResolutionTemp[0];
    config_.imgRows = camResolutionTemp[1];
    vector<double> camIntrinsicsTemp(4);
    nh.getParam("cam/intrinsics", camIntrinsicsTemp);
    config_.camIntrinsics[0] = camIntrinsicsTemp[0];
    config_.camIntrinsics[1] = camIntrinsicsTemp[1];
    config_.camIntrinsics[2] = camIntrinsicsTemp[2];
    config_.camIntrinsics[3] = camIntrinsicsTemp[3];
    vector<double> camDistortionCoeffsTemp(4);
    nh.getParam("cam/distortion_coeffs", camDistortionCoeffsTemp);
    config_.camDistortionCoeffs[0] = camDistortionCoeffsTemp[0];
    config_.camDistortionCoeffs[1] = camDistortionCoeffsTemp[1];
    config_.camDistortionCoeffs[2] = camDistortionCoeffsTemp[2];
    config_.camDistortionCoeffs[3] = camDistortionCoeffsTemp[3];
    nh.param<string>("cam/distortion_model", config_.camDistortionModel, string("radtan"));

    //2. lk tracker parameters
    nh.getParam("point_tracker/max_cnt", config_.maxCnt);
    nh.getParam("point_tracker/min_dist", config_.minDist);
    nh.getParam("point_tracker/equalize", config_.equalize);
    nh.getParam("point_tracker/F_threshold", config_.ransacThreshold);

    //3. print
    ROS_INFO("cam_resolution: %d, %d", camResolutionTemp[0], camResolutionTemp[1]);
    ROS_INFO("cam_intrinscs: %f, %f, %f, %f", camIntrinsicsTemp[0], camIntrinsicsTemp[1], camIntrinsicsTemp[2], camIntrinsicsTemp[3]);
    ROS_INFO("cam_distortion_model: %s", config_.camDistortionModel.c_str());
    ROS_INFO("cam_distortion_coefficients: %f, %f, %f, %f", camDistortionCoeffsTemp[0], camDistortionCoeffsTemp[1], camDistortionCoeffsTemp[2], camDistortionCoeffsTemp[3]);
    ROS_INFO("point features max_cnt: %d", config_.maxCnt);
    ROS_INFO("point features min_distance: %d", config_.minDist);
    ROS_INFO("point features tracker is_equalize: %d", config_.equalize);
    ROS_INFO("point features ransac_threshold: %f", config_.ransacThreshold);

    return true;
}

bool FeatureTracker::Run(const cv::Mat& img) {
    Timer timer;
    timer.start();
    //1. 直方图均衡化
    cv::Mat _img;
    if (config_.equalize) {
        // TODO:: 这里直接apply(img, currImg_)不可行
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, _img);
        currImg_ = _img;
    } else {
        currImg_ = img;
    }

    //2. 初始化第一帧
    if (isFirstImg_) {
        InitializeFirstFrame();
        isFirstImg_ = false;
    } else {
        TrackFeatures();
    }
    ROS_DEBUG("Lk feature tracker costs: %f ms", timer.stop());
    return true;
}

Features& FeatureTracker::GetFeatures() { 
    return currFeatures_; 
}

cv::Mat FeatureTracker::DebugShow() {
    static cv::Matx33d rectifyK;
    static cv::Mat map1, map2;
    static bool initial = false;

    cv::Vec4d& cam_intrinsics = config_.camIntrinsics;
    cv::Vec4d& cam_distortion_coeffs = config_.camDistortionCoeffs;
    cv::Size img_size = currImg_.size();
    cv::Matx33d K(cam_intrinsics[0], 0.0, cam_intrinsics[2], 
                  0.0, cam_intrinsics[1], cam_intrinsics[3], 
                  0.0, 0.0, 1.0);
    if (initial == false) {
        rectifyK = cv::getOptimalNewCameraMatrix(K, cam_distortion_coeffs, img_size, 1, img_size, 0);
        cv::initUndistortRectifyMap(K, cam_distortion_coeffs, cv::Mat(), rectifyK, img_size, CV_16SC2, map1, map2);
        initial = true;
    }
    cv::Mat undistortedImg, currImg;
    cv::remap(currImg_, undistortedImg, map1, map2, cv::INTER_LINEAR);
    // 若不进行色彩空间转换，cv::circle画出来的是灰度点
    cv::cvtColor(currImg_, currImg, CV_GRAY2RGB);
    cv::cvtColor(undistortedImg, undistortedImg, CV_GRAY2RGB);

    for (int i = 0; i < static_cast<int>(currFeatures_.size()); ++i) {
        FeatureMetaData& feature = currFeatures_[i];
        double len = std::min(1.0, 1.0 * feature.lifeTime / 20);
        cv::circle(currImg, feature.ptInImg, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
        cv::circle(undistortedImg, feature.ptInImgRectified, 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
    }

    cv::imshow("LK-RawImg", currImg);
    cv::imshow("LK-UndistortImg", undistortedImg);
    cv::waitKey(5);

    return undistortedImg;
}

void FeatureTracker::InitializeFirstFrame() {
    prevCvFeatures_.clear();
    currCvFeatures_.clear();
    currFeatures_.clear();
    cv::goodFeaturesToTrack(currImg_, currCvFeatures_, config_.maxCnt, 0.1, config_.minDist, mask_);
    cv::Point2f ptInPixelRectifiedTemp;
    cv::Point3f ptInNormalizedCamTemp;
    for (int i = 0; i < currCvFeatures_.size(); ++i) {
        FeatureMetaData featurePointTemp;
        UndistortPoint(currCvFeatures_[i], config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        featurePointTemp.id = nextFeatureId_++;
        featurePointTemp.lifeTime = 1;
        featurePointTemp.ptInImg = currCvFeatures_[i];
        featurePointTemp.ptInNormalizedCam = ptInNormalizedCamTemp;
        featurePointTemp.ptInImgRectified = ptInPixelRectifiedTemp;
        currFeatures_.push_back(featurePointTemp);
    }

    prevImg_ = currImg_;
    prevCvFeatures_ = currCvFeatures_;

    return;
}

void FeatureTracker::TrackFeatures() {
    //1. LK光流跟踪
    vector<uchar> status;
    vector<float> err;
    currCvFeatures_.clear();
    cv::calcOpticalFlowPyrLK(prevImg_, currImg_, prevCvFeatures_, currCvFeatures_, status, err, cv::Size(21, 21), 3);

    //2. 跟踪成功，但是在图像外的点，设置为跟踪失败
    for (int i = 0; i < static_cast<int>(currCvFeatures_.size()); i++)
        if (status[i] && !InBorder(currCvFeatures_[i])) status[i] = 0;

    RemoveUnmarkedElements(prevCvFeatures_, status);
    RemoveUnmarkedElements(currCvFeatures_, status);
    RemoveUnmarkedElements(currFeatures_, status);

    //3. Ransac去除外点
    RejectWithF();

    //4. 对跟踪上的特征点进行更新
    cv::Point2f ptInPixelRectifiedTemp;
    cv::Point3f ptInNormalizedCamTemp;
    for (int i = 0; i < currCvFeatures_.size(); ++i) {
        UndistortPoint(currCvFeatures_[i], config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        FeatureMetaData& featurePointTemp = currFeatures_[i];
        featurePointTemp.lifeTime += 1;
        featurePointTemp.ptInImg = currCvFeatures_[i];
        featurePointTemp.ptInNormalizedCam = ptInNormalizedCamTemp;
        featurePointTemp.ptInImgRectified = ptInPixelRectifiedTemp;
    }

    //5. 跟踪到的特征点不足，重新补充
    SetMask();
    int featureToAddNum = config_.maxCnt - static_cast<int>(currCvFeatures_.size());
    if (featureToAddNum > 0) {
        // 补充一些新的特征点,存储于n_pts容器
        cv::goodFeaturesToTrack(currImg_, newAddPts_, featureToAddNum, 0.1, config_.minDist, mask_);
    } else
        newAddPts_.clear();
    for (auto& it : newAddPts_) {
        currCvFeatures_.push_back(it);
        FeatureMetaData featurePointTemp;
        UndistortPoint(it, config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        featurePointTemp.id = nextFeatureId_++;
        featurePointTemp.lifeTime = 1;
        featurePointTemp.ptInImg = it;
        featurePointTemp.ptInNormalizedCam = ptInNormalizedCamTemp;
        featurePointTemp.ptInImgRectified = ptInPixelRectifiedTemp;
        currFeatures_.push_back(featurePointTemp);
    }

    prevImg_ = currImg_;
    prevCvFeatures_ = currCvFeatures_;

    return;
}

void FeatureTracker::RejectWithF() {
    // Timer timer;
    // timer.start();
    if (currCvFeatures_.size() >= 8) {
        // 对跟踪到的角点去畸变
        vector<cv::Point2f> undistort_prev_pixel_pts(0);
        vector<cv::Point2f> undistort_curr_pixel_pts(0);
        vector<cv::Point3f> undistort_prev_unit_pts(0);
        vector<cv::Point3f> undistort_curr_unit_pts(0);
        UndistortPoints(prevCvFeatures_, config_.camIntrinsics, config_.camDistortionCoeffs,
                        config_.camDistortionModel, undistort_prev_pixel_pts, undistort_prev_unit_pts);
        UndistortPoints(currCvFeatures_, config_.camIntrinsics, config_.camDistortionCoeffs,
                        config_.camDistortionModel, undistort_curr_pixel_pts, undistort_curr_unit_pts);


        // ransac去除外点
        vector<uchar> status;
        cv::findFundamentalMat(undistort_prev_pixel_pts, undistort_curr_pixel_pts, cv::FM_RANSAC, config_.ransacThreshold, 0.99, status);

        RemoveUnmarkedElements(prevCvFeatures_, status);
        RemoveUnmarkedElements(currCvFeatures_, status);
        RemoveUnmarkedElements(currFeatures_, status);
       
        // ROS_DEBUG("FM ransac costs: %f ms", timer.stop());
    }

    return;
}

void FeatureTracker::UndistortPoint(const cv::Point2f& pt_in, 
                                    const cv::Vec4d& cam_intrinsics, 
                                    const cv::Vec4d& cam_distortion_coeffs, 
                                    const string& cam_distortion_model, 
                                    cv::Point2f& pt_out, 
                                    cv::Point3f& pt_normp_out) {

    const cv::Matx33d K(cam_intrinsics[0], 0.0, cam_intrinsics[2], 
                        0.0, cam_intrinsics[1], cam_intrinsics[3], 
                        0.0, 0.0, 1.0);

    cv::Mat matIn(1,1, CV_32FC2, cv::Scalar(pt_in.x, pt_in.y));
    cv::Mat matOut;

    if (cam_distortion_model == "radtan") {
        cv::undistortPoints(matIn, matOut, K, cam_distortion_coeffs);
    } else if (cam_distortion_model == "equidistant") {
        cv::fisheye::undistortPoints(matIn, matOut, K, cam_distortion_coeffs);
    } else {
        ROS_WARN_ONCE("The model %s is unrecognized, use radtan instead...", cam_distortion_model.c_str());
        cv::undistortPoints(matIn, matOut, K, cam_distortion_coeffs);
    }

    pt_normp_out.x = matOut.at<cv::Vec2f>(0,0)[0];
    pt_normp_out.y = matOut.at<cv::Vec2f>(0,0)[1];
    pt_normp_out.z = 1.0;

    pt_out.x = cam_intrinsics[0] * pt_normp_out.x + cam_intrinsics[2];
    pt_out.y = cam_intrinsics[1] * pt_normp_out.y + cam_intrinsics[3];

    return;
}

void FeatureTracker::UndistortPoints(const vector<cv::Point2f>& pts_in, 
                                     const cv::Vec4d& cam_intrinsics, 
                                     const cv::Vec4d& cam_distortion_coeffs, 
                                     const string& cam_distortion_model, 
                                     vector<cv::Point2f>& pts_out, 
                                     vector<cv::Point3f>& pts_normp_out) {
    if (pts_in.size() == 0) return;
    pts_out.clear();
    pts_normp_out.clear();

    const cv::Matx33d K(cam_intrinsics[0], 0.0, cam_intrinsics[2], 
                        0.0, cam_intrinsics[1], cam_intrinsics[3], 
                        0.0, 0.0, 1.0);

    const cv::Matx33d rectification_matrix = cv::Matx33d::eye();

    if (cam_distortion_model == "radtan") {
        cv::undistortPoints(pts_in, pts_out, K, cam_distortion_coeffs, rectification_matrix, K);
    } else if (cam_distortion_model == "equidistant") {
        cv::fisheye::undistortPoints(pts_in, pts_out, K, cam_distortion_coeffs, rectification_matrix, K);
    } else {
        ROS_WARN_ONCE("The model %s is unrecognized, use radtan instead...", cam_distortion_model.c_str());
        cv::undistortPoints(pts_in, pts_out, K, cam_distortion_coeffs, rectification_matrix, K);
    }

    for (auto& it : pts_out) {
        cv::Point3f p;
        p.x = (it.x - cam_intrinsics[2]) / cam_intrinsics[0];
        p.y = (it.y - cam_intrinsics[3]) / cam_intrinsics[1];
        p.z = 1;
        pts_normp_out.push_back(p);
    }

    return;
}

void FeatureTracker::SetMask() {
    mask_ = cv::Mat(config_.imgRows, config_.imgCols, CV_8UC1, cv::Scalar(255));

    // prefer to keep features that are tracked for long time
    vector<pair<int, int>> lifeTime_idx;

    for (unsigned int i = 0; i < currCvFeatures_.size(); i++)
        lifeTime_idx.push_back(make_pair(currFeatures_[i].lifeTime, i));

    // 按照跟踪次数降顺排列
    sort(lifeTime_idx.begin(), lifeTime_idx.end(), [](const pair<int, int>& a, const pair<int, int>& b) 
            { 
                return a.first > b.first; 
            });

    for (auto& it : lifeTime_idx) {
        if(mask_.at<uchar>(currFeatures_[it.second].ptInImg) == 255){
            cv::circle(mask_, currFeatures_[it.second].ptInImg, config_.minDist, 0, -1);
        }
    }

    return;
}

bool FeatureTracker::InBorder(const cv::Point2f& pt) {
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    return BORDER_SIZE <= img_x && img_x < config_.imgCols - BORDER_SIZE && 
           BORDER_SIZE <= img_y && img_y < config_.imgRows - BORDER_SIZE;
}

}  // end namespace CIL
