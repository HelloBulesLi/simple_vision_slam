#pragma once

#ifndef MYSLAM_VIEWER_H
#define MYSLAM_VIEWER_H

#include <thread>
#include "frame.hpp"
#include "map.hpp"
#include <pangolin/pangolin.h>

namespace myslam {

// Class Viewer used to show the local map and Keyframes's traj in Sliding Window

class Viewer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Viewer> Ptr;

    Viewer();

    void SetMap(Map::Ptr map) { map_ = map;}

    void Close();

    // add a current frame
    void AddCurrentFrame(Frame::Ptr current_frame);

    // update map
    void UpdateMap();

    // for debug
    cv::Mat ShowImgKpForDebug(Frame::Ptr frame);

private:
    void ThreadLoop();

    void DrawFrame(Frame::Ptr frame, const float* color);

    void DrawMapPoints();

    void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera);

    /// plot the feature in current frame into an image
    cv::Mat PlotFrameImage();
    Frame::Ptr current_frame_ = nullptr;
    Map::Ptr map_ = nullptr;

    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;

};

}

#endif
