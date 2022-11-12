#pragma once

#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include "camera.hpp"
// #include "myslam/common_include.hpp"
#include "feature.hpp"
#include "frame.hpp"
#include "map.hpp"
#include "mappoint.hpp"
#include "viewer.hpp"
#include "backend.hpp"
#include <fstream>

namespace myslam {

using namespace cv;
using namespace std;

enum class Frontend_state {INIT_STATE,TRACK_GOOD, TRACK_BAD, TRACK_LOST};

// Class Frontend's main function is:
// 1. init map and track feature in lastframe
// 2. get a pose estimate from tracked mappoint in current frame
// 3. decide wether current frame is a keyframe
// 4. add new observation to mappoint and detect new mappoint
class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;


    Frontend();

    bool AddFrame(Frame::Ptr);
    void SetMap(Map::Ptr map) {
        map_ = map;
    }
    // void SetBackend(std::shared_ptr<Backend> backend);
    // void SetViewer(std::shared_ptr<Viewer> Viewer);
    void SetCamera(Camera::Ptr left_camera, Camera::Ptr right_camera)
    {
        left_camera_ = left_camera;
        right_camera_ = right_camera;
    }

    void SetBackend(Backend::Ptr backend)
    {
        backend_ = backend;
    }

    void SetViewer(Viewer::Ptr viewer)
    {
        viewer_ = viewer;
    }

    Frontend_state GetStatus() {return state_;}
private:
    void StereoInit();
    void Track();
    int TrackLastframe();

    int EstimatePose();
    void DetectFeature();
    void FindMatchInRight();
    int BuildInitMap();
    void InsertKeyframe();
    void AddObservations();
    void Reset();

    void TriangulateNewMapponit();
    bool triangulate(Vector3d point_left, Vector3d point_right, 
                    Sophus::SE3d pose_left, Sophus::SE3d pose_right, Vector3d &pw);

    Frame::Ptr current_frame_;
    Frame::Ptr last_frame_;
    Sophus::SE3d rel_velocity_ = Sophus::SE3d();
    Frontend_state state_ = Frontend_state::INIT_STATE;

    int initial_map_point = 50;

    int track_min_mapponit = 20;

    int track_good_mappoint = 50;

    int track_max_mappoint = 80;

    int track_inliner_mappoint_ = 0;

    int active_keyframe_max_ = 20;


    cv::Ptr<cv::FeatureDetector> feature_detector_;
    cv::Ptr<cv::GFTTDetector> gftt_;
    Camera::Ptr left_camera_;
    Camera::Ptr right_camera_;
    Map::Ptr map_;
    Backend::Ptr backend_;
    Viewer::Ptr viewer_;
    int active_keyframe_num = 0;
    bool have_reset_ = false;
    std::ofstream traj_record_;

};

}

#endif
