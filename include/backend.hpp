#pragma once

#ifndef MY_SLAM_BACKEND_H
#define MY_SLAM_BACKEND_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <iostream>
#include <thread>
#include "map.hpp"
#include "frame.hpp"
#include "camera.hpp"
#include <condition_variable>

namespace myslam {

// Class Backend finish the sliding window of keyframes, this optimization
// can't used in real time, only used for optimize the local trajectory

class Backend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Backend> Ptr;
    Backend();
    void BackendLoop();
    void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    void SetMap(Map::Ptr map) {
        map_ = map;
    }

    void SetCamera(Camera::Ptr left_camera, Camera::Ptr right_camera)
    {
        left_camera_ = left_camera;
        right_camera_ = right_camera;
    }

    // every time map update, call this interface to trigger a global BA
    void UpdateMap();

private:
    Map::Ptr map_;
    std::thread backend_thread_;
    Map::KeyframesType active_keyframes_;
    Map::LandmarksType active_landmarks_;
    Camera::Ptr left_camera_;
    Camera::Ptr right_camera_;
    std::condition_variable map_update_;
    std::mutex data_mutex_;
    std::ofstream log_record;
    int global_BA_cnt = 0;
};

}



#endif
