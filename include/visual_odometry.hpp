#pragma once

#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "camera.hpp"
// #include "myslam/common_include.hpp"
#include "feature.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "frontend.hpp"
#include "dataset.hpp"
#include "viewer.hpp"
#include "backend.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <glog/logging.h>

namespace myslam {

class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    VisualOdometry(std::string config_file_path);

    /**
     * @brief do initialization befor run
     * 
     * @return true 
     * @return false 
     */
    bool Init();

    /**
     * @brief start vo in the dataset
     * 
     */
    void Run();

    /**
     * @brief Make a step forward in dateset
     * 
     */
    bool Step();

    /// gian the front end state
    Frontend_state GetFrontendStatus() const { return frontend_->GetStatus();}
    
    // access the parameter values
    // template <typename T>
    // static T Get(const std::string &key) {
    //     return T(file_[key]);
    // }

private:
    // use dataset to read img file
    Dataset::Ptr dataset_ = nullptr;
    std::string config_file_path_;
    Frontend::Ptr frontend_;
    Map::Ptr map_;
    cv::FileStorage file_;
    Viewer::Ptr viewer_;
    Backend::Ptr backend_;
};

}

#endif
