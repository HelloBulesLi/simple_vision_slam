#pragma once

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include "frame.hpp"
#include <pthread.h>
#include <mutex>
#include <memory>

namespace myslam {

using namespace std;
using namespace cv;
using namespace Eigen;

class Feature;

class MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;
    int observed_times_ = 0;
    Sophus::Vector3d pw_;

    MapPoint() {}
    MapPoint(unsigned long id, Sophus::Vector3d &pw);

    Sophus::Vector3d Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pw_;
    }

    void SetPos(Sophus::Vector3d &pw) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pw_ = pw;
    }

    std::list<std::weak_ptr<Feature>> observation;

    void AddObservations(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observation.push_back(feature);
        observed_times_++;
    }
    
    void RemoveObservation(std::shared_ptr<Feature> feature);

    static MapPoint::Ptr CreateNewMappoint();
private:
    std::mutex data_mutex_;
    
};

}

#endif
