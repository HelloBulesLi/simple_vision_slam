#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

// #include "feature.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>

namespace myslam {

using namespace std;
using namespace cv;
class Feature;

class Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;
    Frame() {}
    Frame(long id, double time_stamp, const Sophus::SE3d &pose, const Mat &left,
         const Mat &right);
    std::mutex pose_mutex_;
    cv::Mat left_img_;
    cv::Mat right_img_;
    vector<std::shared_ptr<Feature>> left_features_;
    vector<std::shared_ptr<Feature>> right_features_;

    Sophus::SE3d pose_;
    bool is_keyframe_ = false;
    double time_stamp_;
    int keyframe_id_ = 0;
    int id_;
    static Frame::Ptr CreateFrame();
    void SetKeyFrame();

    void SetPose(const Sophus::SE3d &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    Sophus::SE3d Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }
};

}

#endif