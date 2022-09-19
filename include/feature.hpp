#pragma once

#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
// #include "mappoint.hpp"

namespace myslam {

class Frame;
class MapPoint;
using namespace std;
using namespace cv;
class Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;
    Feature() {}
    cv::KeyPoint kp_pos_;
    std::weak_ptr<Frame> frame_;
    std::weak_ptr<MapPoint> mp_;
    bool is_outlier_ = false;
    bool is_on_left_img_ = true;
};

}

#endif
