/**
 * @file linefeature_tracker.cc
 * @brief 线特征提取与跟踪类
 * @author GWH
 * @version 0.1
 * @date 2021-04-07 08:51:55
 */
#include <cil-slam/linefeature_tracker.h>
#include <cil-slam/timer.h>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>

namespace CIL {

LineFeatureTracker::LineFeatureTracker() : isFirstImg_(true), nextFeatureId_(0) {
    ROS_INFO("Line feature tracker begin ==============================");
    lsd_ = cv::line_descriptor::LSDDetector::createLSDDetector();
    lbd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    lineMatcher_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

    return;
}

LineFeatureTracker::LineFeatureTracker(ros::NodeHandle& nh) : nh_(nh), isFirstImg_(true), nextFeatureId_(0) {
    ROS_INFO("Line feature tracker begin ==============================");
    LoadParameters(nh_);
    lsd_ = cv::line_descriptor::LSDDetector::createLSDDetector();
    lbd_ = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
    lineMatcher_ = cv::line_descriptor::BinaryDescriptorMatcher::createBinaryDescriptorMatcher();

    return;
}

bool LineFeatureTracker::LoadParameters(const ros::NodeHandle& nh) {
    // 1.camera calibration parameters from config/config.yaml
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

    // 2.line tracker parameters
    nh.getParam("line_tracker/max_cnt", config_.maxCnt);
    nh.getParam("line_tracker/scale", config_.scale);
    nh.getParam("line_tracker/octaves", config_.octaves);
    nh.getParam("line_tracker/min_line_length", config_.minLineLength);
    nh.getParam("line_tracker/line_link_diff_distance", config_.lineLinkDistance);
    nh.getParam("line_tracker/line_link_diff_angle", config_.lineLinkAngle);
    nh.getParam("line_tracker/line_merge_diff_distance", config_.lineMergeDistance);
    nh.getParam("line_tracker/max_match_distance", config_.goodMatchesDistance);

    // 3.print
    ROS_INFO("line features max_cnt: %d", config_.maxCnt);
    ROS_INFO("lsd scale: %f", config_.scale);
    ROS_INFO("lsd octaves: %d", config_.octaves);
    ROS_INFO("line features min_line_length: %d", config_.minLineLength);
    ROS_INFO("line featues max link distance: %f", config_.lineLinkDistance);
    ROS_INFO("line featues max link angle: %f", config_.lineLinkAngle);
    ROS_INFO("line featues max merge distance: %f", config_.lineMergeDistance);
    ROS_INFO("lbd gooe matches max distance: %f", config_.goodMatchesDistance);

    // 4.畸变映射计算
    cv::Vec4d& cam_intrinsics = config_.camIntrinsics;
    cv::Vec4d& cam_distortion_coeffs = config_.camDistortionCoeffs;
    cv::Size img_size = cv::Size(config_.imgCols, config_.imgRows);
    cv::Matx33d K(cam_intrinsics[0], 0.0, cam_intrinsics[2], 
                  0.0, cam_intrinsics[1], cam_intrinsics[3], 
                  0.0, 0.0, 1.0);
    cv::Matx33d rectifyK = cv::getOptimalNewCameraMatrix(K, cam_distortion_coeffs, img_size, 1, img_size, 0);
    cv::initUndistortRectifyMap(K, cam_distortion_coeffs, cv::Mat(), rectifyK, img_size, CV_16SC2, undistMap1_, undistMap2_);

    return true;
}

bool LineFeatureTracker::Run(const cv::Mat& img) {
    Timer timer;
    timer.start();
    // 1.直方图分布均衡化
    cv::Mat _img;
    if (config_.equalize) {
        // TODO:: 这里直接apply(img, currImg_)不可行
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
        clahe->apply(img, _img);
        currImg_ = _img;
    } else {
        currImg_ = img;
    }
    // TODO::2. 整张图像去除畸变（使用的相机畸变较小，这里先注释掉）
    // cv::remap(currImg_, currImg_, undistMap1_, undistMap2_, cv::INTER_LINEAR);
    if(isFirstImg_){
        InitializeFirstFrame();
        isFirstImg_ = false;
    }else{
        Matching();
    }
    ROS_DEBUG("Line feature tracker costs: %f ms", timer.stop());
    return true;
}

LineFeatures& LineFeatureTracker::GetFeatures() { 
    return currLines_; 
}

cv::Mat LineFeatureTracker::DebugShow() {
    cv::Mat undistortedImg;
    cv::remap(currImg_, undistortedImg, undistMap1_, undistMap2_, cv::INTER_LINEAR);
    cv::cvtColor(undistortedImg, undistortedImg, CV_GRAY2RGB);
    for (int i = 0; i < static_cast<int>(currLines_.size()); ++i) {
        LineFeatureMetaData& LineFeatures = currLines_[i];
        if(LineFeatures.lifeTime > 1) {
            cv::line(undistortedImg, LineFeatures.startPointRectified, LineFeatures.endPointRectified,
                     cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
        }else{
            cv::line(undistortedImg, LineFeatures.startPointRectified, LineFeatures.endPointRectified,
                     cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
        }
    }
    cv::imshow("Line-UndistortImg", undistortedImg);
    cv::waitKey(5);

    return undistortedImg;
}

void LineFeatureTracker::InitializeFirstFrame() {
    DetectNewLine();
    // 转换为自定义的线段结构
    cv::Point2f ptInPixelRectifiedTemp;
    cv::Point3f ptInNormalizedCamTemp;
    for (KeyLine& it : newAddLines_) {
        LineFeatureMetaData lineFeature;
        lineFeature.id = nextFeatureId_++;
        lineFeature.lifeTime = 1;
        lineFeature.startPoint = cv::Point2f(it.startPointX, it.startPointY);
        lineFeature.endPoint = cv::Point2f(it.endPointX, it.endPointY);
        UndistortPoint(lineFeature.startPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        lineFeature.startPointRectified = ptInPixelRectifiedTemp;
        UndistortPoint(lineFeature.endPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        lineFeature.endPointRectified = ptInPixelRectifiedTemp;
        currLines_.push_back(lineFeature);
    }

    prevImg_ = currImg_;
    prevCvLines_ = newAddLines_;
    prevLinesDescriptor_ = newAddLinesDescriptor_;

    return;
}

void LineFeatureTracker::Matching() {
    currCvLines_.clear();
    currLinesDescriptor_.release();
    DetectNewLine();

    // 1.前后帧线段匹配
    vector<cv::DMatch> lsdMatches;
    if(prevCvLines_.size() > 0) {
        lineMatcher_->match(prevLinesDescriptor_, newAddLinesDescriptor_, lsdMatches);
    }

    // 2.选取质量较好的匹配对
    vector<cv::DMatch> goodMatches;
    vector<uchar> currStatus(prevCvLines_.size(), 0);
    for (int i = 0; i < static_cast<int>(lsdMatches.size()); ++i) {
        if (lsdMatches[i].distance < config_.goodMatchesDistance) {
            const cv::DMatch& mt = lsdMatches[i];
            KeyLine line1 = prevCvLines_[mt.queryIdx];
            KeyLine line2 = newAddLines_[mt.trainIdx];
            cv::Point2f serr = line1.getStartPoint() - line2.getStartPoint();
            cv::Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
            // 确保线段在图像里不会跑得特别远
            if ((serr.dot(serr) < 60 * 60) && (eerr.dot(eerr) < 60 * 60)) {
                goodMatches.push_back(mt);
                currStatus[mt.queryIdx] = 1;
            }
        }
    }

    // 3.更新跟踪上的线段
    cv::Point2f ptInPixelRectifiedTemp;
    cv::Point3f ptInNormalizedCamTemp;
    for (int i = 0; i < static_cast<int>(goodMatches.size()); ++i) {
        const cv::DMatch& mt = goodMatches[i];
        LineFeatureMetaData& lineFeature = currLines_[mt.queryIdx];
        lineFeature.lifeTime += 1;
        lineFeature.startPoint.x = newAddLines_[mt.trainIdx].startPointX;
        lineFeature.startPoint.y = newAddLines_[mt.trainIdx].startPointY;
        lineFeature.endPoint.x = newAddLines_[mt.trainIdx].endPointX;
        lineFeature.endPoint.y = newAddLines_[mt.trainIdx].endPointY;
        UndistortPoint(lineFeature.startPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        lineFeature.startPointRectified = ptInPixelRectifiedTemp;
        UndistortPoint(lineFeature.endPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                       config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
        lineFeature.endPointRectified = ptInPixelRectifiedTemp;

        currCvLines_.push_back(newAddLines_[mt.trainIdx]);
        currLinesDescriptor_.push_back(newAddLinesDescriptor_.row(mt.trainIdx));
    }
    RemoveUnmarkedElements(currLines_, currStatus);

    // 4.跟踪上的线段不足，将其余检测出来的线段补充进去
    int diff = config_.maxCnt - currLines_.size();
    if (diff > 0) {
        for (int i = 0; i < static_cast<int>(newAddLines_.size()); ++i) {
            if (currStatus[i] == 0) {
                KeyLine& cvLine = newAddLines_[i];
                LineFeatureMetaData lineFeature;
                lineFeature.id = nextFeatureId_++;
                lineFeature.lifeTime = 1;
                lineFeature.startPoint.x = cvLine.startPointX;
                lineFeature.startPoint.y = cvLine.startPointY;
                lineFeature.endPoint.x = cvLine.endPointX;
                lineFeature.endPoint.y = cvLine.endPointY;
                UndistortPoint(lineFeature.startPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                               config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
                lineFeature.startPointRectified = ptInPixelRectifiedTemp;
                UndistortPoint(lineFeature.endPoint, config_.camIntrinsics, config_.camDistortionCoeffs,
                               config_.camDistortionModel, ptInPixelRectifiedTemp, ptInNormalizedCamTemp);
                lineFeature.endPointRectified = ptInPixelRectifiedTemp;
                currLines_.emplace_back(lineFeature);

                currCvLines_.push_back(cvLine);
                currLinesDescriptor_.push_back(newAddLinesDescriptor_.row(i));
            }
        }
    }

    prevImg_ = currImg_;
    prevCvLines_ = currCvLines_;
    prevLinesDescriptor_ = currLinesDescriptor_;

    return;
}

void LineFeatureTracker::DetectNewLine() {
    // 1.检测线段
    newAddLines_.clear();
    newAddLinesDescriptor_.release();
    lsd_->detect(currImg_, newAddLines_, config_.scale, config_.octaves);
    // lbd_->compute(currImg_, newAddLines_, newAddLinesDescriptor_);
    // 2. 保留高响应值的线段
    if (newAddLines_.size() > config_.maxCnt && config_.maxCnt != 0) {
        // sort lines by their response
        sort(newAddLines_.begin(), newAddLines_.end(), [](const KeyLine& a, const KeyLine& b) 
                { 
                    return a.response > b.response; 
                });
        newAddLines_.resize(config_.maxCnt);
    }

    // 3.去除长度过短的线段
    vector<KeyLine> keyLinesTemp;
    int id = 0;
    for (KeyLine& cvLine : newAddLines_) {
        if(cvLine.octave == 0 && cvLine.lineLength >= config_.minLineLength) {
            cvLine.class_id = id++;
            keyLinesTemp.push_back(cvLine);
        }
    }

    // 4.更新描述子（先计算描述子在过滤与先过滤再计算描述子效果相同）
    // cv::Mat keyLinesDescriptor;
    // for (KeyLine& cvLine : keyLinesTemp) {
    //     keyLinesDescriptor.push_back(newAddLinesDescriptor_.row(cvLine.class_id));
    // }
    newAddLines_ = move(keyLinesTemp);
    // newAddLinesDescriptor_ = keyLinesDescriptor;
    
    lbd_->compute(currImg_, newAddLines_, newAddLinesDescriptor_);
    return;
}

void LineFeatureTracker::UndistortPoint(const cv::Point2f& pt_in, 
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

} /* CIL  */ 
