#pragma once

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include "feature.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include "frame.hpp"
#include "mappoint.hpp"

namespace myslam {

using namespace std;
using namespace cv;
class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframesType all_keyframes_;
    KeyframesType active_keyframes_;
    std::mutex map_data_mutex_;

    std::vector<std::weak_ptr<Feature>> observation;

    void InsertMapPoint(MapPoint::Ptr map_point);
    // what diff between insert and direct use operator [] ?
    void InsertKeyframe(Frame::Ptr frame);

    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(map_data_mutex_);
        return active_keyframes_;
    }

    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(map_data_mutex_);
        return active_landmarks_;
    }

    void RemoveOldKeyframe();
    
    void ResetMap();

    void CleanMap();
private:
    Frame::Ptr current_frame_ = nullptr;
    int num_active_keyframes_ = 7;
};

}

#endif
