#include "frontend.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include "feature.hpp"
#include "g2o_types.hpp"
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/types/slam3d/edge_se3_prior.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

namespace myslam {

Frontend::Frontend()
{
    // CV_WRAP static Ptr<ORB> create(int nfeatures=500, float scaleFactor=1.2f, int nlevels=8, int edgeThreshold=31,
    //     int firstLevel=0, int WTA_K=2, int scoreType=ORB::HARRIS_SCORE, int patchSize=31, int fastThreshold=20);
    // feature_detector_ = ORB::create(200, 2, 4, 31, 0, 2, ORB::HARRIS_SCORE, 31, 30);
    gftt_ = cv::GFTTDetector::create(150, 0.01, 20);
        // CV_WRAP static Ptr<GFTTDetector> create( int maxCorners=1000, double qualityLevel=0.01, double minDistance=1,
        //                                      int blockSize=3, bool useHarrisDetector=false, double k=0.04 );
}
bool Frontend::AddFrame(Frame::Ptr frame)
{
    current_frame_ = frame;
    // judge the frontend state
    switch(state_) {
        case Frontend_state::INIT_STATE:
        {
            StereoInit();
            break;
        }
        case Frontend_state::TRACK_GOOD:
        case Frontend_state::TRACK_BAD:
        {
            Track();
            break;
        }
        case Frontend_state::TRACK_LOST:
        {
            Reset();
            break;
        }
    }
    last_frame_ = current_frame_;

    return true;
}

void Frontend::Track()
{
    // projection last frame's mappoint to current_frame 
    auto t1 = std::chrono::steady_clock::now();
    TrackLastframe();
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = 
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    
    //for debug
    // LOG(INFO) << "VO track last frame cost: " << time_used.count() << " secons.";

    traj_record_.open("traj_result.txt", ios::app|ios::out);

    // use mappoint to optimize pose
    t1 = std::chrono::steady_clock::now();
    track_inliner_mappoint_ = EstimatePose();
    t2 = std::chrono::steady_clock::now();
    time_used = 
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    
    //for debug
    // LOG(INFO) << "VO track esti pose cost: " << time_used.count() << " secons.";

    // for debug
    LOG(INFO) << "current inliner track num is " << track_inliner_mappoint_ << endl;
    LOG(INFO) << "current frame pose is " << endl << current_frame_->Pose().inverse().matrix();

    traj_record_ << current_frame_->Pose().inverse().matrix().row(0) << ' '
                 << current_frame_->Pose().inverse().matrix().row(1) << ' '
                 << current_frame_->Pose().inverse().matrix().row(2) << endl;

    traj_record_.close();
    // judge current_frame is a keyframe by tracked mappoint inliner count
    if (track_inliner_mappoint_ > track_good_mappoint)
    {
        state_ = Frontend_state::TRACK_GOOD;
    }
    else if(track_inliner_mappoint_ > track_min_mapponit)
    {
        state_ = Frontend_state::TRACK_BAD;
    }
    else
    {
        // for debug
        if (viewer_)
        {
            cv::Mat cur_img = viewer_->ShowImgKpForDebug(current_frame_);
            cv::imshow("curren frame and kp", cur_img);
            cv::Mat last_img = viewer_->ShowImgKpForDebug(last_frame_);
            cv::imshow("last frame and kp", last_img);
            waitKey(0);
        }
        state_ = Frontend_state::TRACK_LOST;
    }

    // after track lost , should direct return ?
    if (state_ == Frontend_state::TRACK_LOST)
    {
        return ;
    }
    // insert keyframe:
    // 1. jude the current frame is a keyframe
    // 2. extract features in current frame and triangulate
    InsertKeyframe();

    // for VO visualization

    if (viewer_) viewer_->AddCurrentFrame(current_frame_);

}

void Frontend::InsertKeyframe()
{
    if (track_inliner_mappoint_ > track_max_mappoint) {
        return ;
    }

    active_keyframe_num++;

    // if (active_keyframe_num > active_keyframe_max_)
    // {
    //     map_->RemoveOldKeyframe();
    // }

    current_frame_->SetKeyFrame();

    map_->InsertKeyframe(current_frame_);

    // add current feature to mapponit obserabation
    AddObservations();

    DetectFeature();

    FindMatchInRight();

    TriangulateNewMapponit();

    // if use backend_, notify the backend thread do global BA when map updated
    // used for only use left obeserve do global BA
    // if(backend_ && (active_keyframe_num > 2))
    if(backend_)
    {
        LOG(INFO) << "trigger a global map";
        backend_->UpdateMap();
    }

    // for VO visualization
    if (viewer_) viewer_->UpdateMap();

    LOG(INFO) << "current frame id is " << current_frame_->id_
            << " keyframe id is " << current_frame_->keyframe_id_;
}

void Frontend::AddObservations()
{
    for(auto feat : current_frame_->left_features_)
    {
        auto mp = feat->mp_.lock();
        if(mp)
        {
            mp->AddObservations(feat);
        }
    }
}

int Frontend::TrackLastframe()
{
    if (last_frame_)
    {
        // current_frame_->SetPose(rel_velocity_*last_frame_->Pose());
        current_frame_->SetPose(last_frame_->Pose());
    }

    // project mappoint to current_frame and track
    vector<cv::Point2f> points_cur,points_last;
    // for (auto feat: last_frame_->left_features_)
    // {
    //     if (feat->mp_.lock())
    //     {
    //         auto mp = feat->mp_.lock();
    //         auto px = left_camera_->world2pixel(mp->pw, current_frame_->pose_);
    //         points_last.push_back(cv::Point2f(feat->kp_pos_.pt));
    //         points_cur.push_back(cv::Point2f(px(0), px(1)));
    //     }
    //     else
    //     {
    //         points_last.push_back(cv::Point2f(feat->kp_pos_.pt));
    //         points_cur.push_back(cv::Point2f(feat->kp_pos_.pt));
    //     }
    // }

    for (auto feat: last_frame_->left_features_)
    {
        points_last.push_back(cv::Point2f(feat->kp_pos_.pt));
        points_cur.push_back(cv::Point2f(feat->kp_pos_.pt));
    }

    vector<uchar> status;
    vector<float> err;
    // cv::Size winSize(31,31);
    cv::Size winSize(11,11);
    cv::TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, points_last, points_cur, status, err, 
                            winSize, 3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 1e-3);

    for(uint32_t i = 0; i < points_cur.size(); i++)
    {
        if(status[i])
        {
            Feature::Ptr feat = Feature::Ptr(new Feature());
            feat->kp_pos_.pt = points_cur[i];
            feat->frame_ = current_frame_;
            feat->mp_ = last_frame_->left_features_[i]->mp_;
            current_frame_->left_features_.push_back(feat);
        }
    }

    LOG(INFO) << "last frame kp track in current frame is " << current_frame_->left_features_.size();
}

int Frontend::EstimatePose()
{
    Sophus::SE3d estimate_pose = Sophus::SE3d();
    // use current frame's mapponit reprojection error to opitimzie pose
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
                  g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType_Cur>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    // optimizer.setVerbose(true);

    // add vertex
    VertexSophus *pose = new VertexSophus();
    pose->setEstimate(current_frame_->Pose());
    pose->setId(0);
    optimizer.addVertex(pose);

    Eigen::Matrix<double,2,2> Information;
    Information.setIdentity();
    
    // add edge
    int index = 1;
    std::vector<OnlyForPoseProjectionEdge *> cur_edges;
    for(uint32_t i = 0; i < current_frame_->left_features_.size(); i++)
    {
        auto feat = current_frame_->left_features_[i];
        auto mp = feat->mp_.lock();
        if (mp)
        {
            // OnlyForPoseProjectionEdge(Matrix<double,3,3> K, Vector3d &Pr)
            // Sophus::Vector3d pw = mp->Pos();
            // OnlyForPoseProjectionEdge *edge = new OnlyForPoseProjectionEdge(left_camera_->K(), pw);
            OnlyForPoseProjectionEdge *edge = new OnlyForPoseProjectionEdge(left_camera_->K(), mp->pw_);
            // edge->setId(mp->id_);
            edge->setId(index);
            edge->setVertex(0, pose);
            edge->setMeasurement(Vector2d(feat->kp_pos_.pt.x, feat->kp_pos_.pt.y));
            edge->setInformation(Information); //information matrix, the inverse of coverance matrix
            edge->setRobustKernel(new g2o::RobustKernelHuber());
            optimizer.addEdge(edge);
            cur_edges.push_back(edge);
            index++;
        }
        else
        {
            cur_edges.push_back(nullptr);
        }
    }

    // for debug
    // LOG(INFO) << "current opt pose use mapppoint " << index << endl;

    
    int iter_count = 0;
    int iter_max = 4;
    // const double chi2_th = 5.991*4;
    // for only use left observation do global BA
    // const double chi2_th = 3*5.991;
    const double chi2_th = 5.991;
    int outliner_count = 0;
    int inliner_count = 0;

    while(iter_count < iter_max)
    {
        // pose->setEstimate(Sophus::SE3d());
        pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(20);
        
        outliner_count = 0;
        inliner_count = 0;

        for (uint32_t i = 0; i < cur_edges.size(); i++)
        {
            if(cur_edges[i])
            {
                if (current_frame_->left_features_[i]->is_outlier_)
                {
                    cur_edges[i]->computeError();
                }

                if (cur_edges[i]->chi2() > chi2_th)
                {
                    current_frame_->left_features_[i]->is_outlier_ = true;
                    cur_edges[i]->setLevel(1);
                    outliner_count++;
                }
                else
                {
                    current_frame_->left_features_[i]->is_outlier_ = false;
                    cur_edges[i]->setLevel(0);
                    inliner_count++;
                }

                if (iter_count == 2)
                {
                   cur_edges[i]->setRobustKernel(nullptr);
                }
            }
        }
        iter_count++;
    }

    double second_chi2_th = 5.991;
    if (inliner_count < track_min_mapponit)
    {
        for (int iteration = 0; iteration < 3; ++iteration) {
            pose->setEstimate(current_frame_->Pose());
            optimizer.initializeOptimization();
            optimizer.optimize(20);
            
            outliner_count = 0;
            inliner_count = 0;

            for (uint32_t i = 0; i < cur_edges.size(); i++)
            {
                if(cur_edges[i])
                {
                    if (current_frame_->left_features_[i]->is_outlier_)
                    {
                        cur_edges[i]->computeError();
                    }

                    if (cur_edges[i]->chi2() > second_chi2_th)
                    {
                        current_frame_->left_features_[i]->is_outlier_ = true;
                        cur_edges[i]->setLevel(1);
                        outliner_count++;
                    }
                    else
                    {
                        current_frame_->left_features_[i]->is_outlier_ = false;
                        cur_edges[i]->setLevel(0);
                        inliner_count++;
                    }
                }
            }

            if (inliner_count < track_min_mapponit)
            {
                second_chi2_th = 2*second_chi2_th;
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << outliner_count << "/"
                << inliner_count << "cure chi2_th is " << second_chi2_th;
    current_frame_->SetPose(pose->estimate());

    // remove the mp obs, if current frame is chosen to be keyframe, this feature 
    // be triangulated to new mapponit, is possible to cause map overlap ?
    for (auto &feat: current_frame_->left_features_) {
        if(feat->is_outlier_) {
            feat->mp_.reset();
            // feat->is_outlier_ = false;
        }
    }

    if (inliner_count > track_min_mapponit)
    {
        rel_velocity_ = current_frame_->Pose()*last_frame_->Pose().inverse();
    }
    else
    {
        rel_velocity_ = Sophus::SE3d();
    }
    
    return inliner_count;
}

// add viewer to display the frontend track result

void Frontend::StereoInit()
{
    // open file to record traj

    // triangulate two initial frame
    std::vector<cv::KeyPoint> keypoints;
    DetectFeature();
    FindMatchInRight();
    int initial_map_num = BuildInitMap();
    if (initial_map_num >= initial_map_point)
    {
        state_ = Frontend_state::TRACK_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
    }
    else
    {
        LOG(INFO) << "current_frame " << current_frame_->id_ << " is not enough mapponit";
        state_ = Frontend_state::INIT_STATE;
    }
}

void Frontend::DetectFeature()
{
    // use ORB detector extract keypoint
    // mask the area already find the mapponit
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for (auto &feat : current_frame_->left_features_) {
        cv::rectangle(mask, feat->kp_pos_.pt -cv::Point2f(10,10),
                        feat->kp_pos_.pt + cv::Point2f(10,10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    // feature_detector_->detect(current_frame_->left_img_, keypoints, mask);
    gftt_->detect(current_frame_->left_img_, keypoints, mask);

    // for debug
    // display the feature extract result
    // Mat outimg1;
    // cv::drawKeypoints( current_frame_->left_img_, keypoints, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // cv::imshow("ORB_FEATURE",outimg1);
    // waitKey(0);

    for(auto kp:keypoints) {
        Feature::Ptr feat = Feature::Ptr(new Feature());
        feat->kp_pos_ = kp;
        feat->frame_ = current_frame_;
        feat->mp_.reset();
        current_frame_->left_features_.push_back(feat);
    }
}

void Frontend::FindMatchInRight()
{
    // projection the mappoint to right img
    vector<cv::Point2f> points_left, points_right;
    for(auto feat:current_frame_->left_features_) {
        auto mp = feat->mp_.lock();
        points_left.push_back(feat->kp_pos_.pt);
        if(mp)
        {
            // Vector2d pixel = right_camera_->world2pixel(mp->pw, current_frame_->pose_.inverse());
            // Sophus::Vector3d pw = mp->Pos();
            // Vector2d pixel = right_camera_->world2pixel(pw, current_frame_->Pose());
            Vector2d pixel = right_camera_->world2pixel(mp->pw_, current_frame_->Pose());
            cv::Point2f cur_pixel = cv::Point2f(pixel(0), pixel(1));
            points_right.push_back(cur_pixel);
        } 
        else
        {
             points_right.push_back(feat->kp_pos_.pt);
        }
    }

    // use LK track
    vector<uchar> status;
    vector<float> err;
    // cv::Size winSize(31,31);
    cv::Size winSize(11,11);
    cv::TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_, points_left, points_right, status, err, 
                            winSize, 3, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW, 1e-3);

    std::vector<cv::KeyPoint> diskeypoints;

    for(uint32_t i = 0; i < points_right.size(); i++)
    {
        if(status[i])
        {
            Feature::Ptr feat = Feature::Ptr(new Feature());
            feat->kp_pos_.pt = points_right[i];
            diskeypoints.push_back(feat->kp_pos_);
            feat->frame_ = current_frame_;
            feat->is_on_left_img_ = false;
            current_frame_->right_features_.push_back(feat);
        }
        else
        {
            current_frame_->right_features_.push_back(nullptr);
        }
    }

    // for debug
    // LOG(INFO) << "right camera track success point num " << current_frame_->right_features_.size();

    // for debug
    // display the right img track result
    // Mat outimg1;
    // cv::drawKeypoints( current_frame_->right_img_, diskeypoints, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    // cv::imshow("Right_TRACK_FEATURE",outimg1);
    // waitKey(0);
}

void Frontend::TriangulateNewMapponit()
{
     // triangulated match point in right and left img
    int map_point_num = 0;
    for (uint32_t i = 0; i < current_frame_->left_features_.size(); i++)
    {
        auto feat = current_frame_->left_features_[i];
        if (feat->is_outlier_)
        {
            continue;
        }

        if((feat->mp_.expired()) && (current_frame_->right_features_[i]))
        {
            auto feat_r = current_frame_->right_features_[i];
            Vector3d left_pc = left_camera_->pixel2camera(
                                        Vector2d(feat->kp_pos_.pt.x, feat->kp_pos_.pt.y));
            Vector3d right_pc = right_camera_->pixel2camera(
                                        Vector2d(feat_r->kp_pos_.pt.x, feat_r->kp_pos_.pt.y));
            Vector3d pw;
            bool ret = triangulate(left_pc, right_pc, left_camera_->pose_, right_camera_->pose_, pw);
            if (ret && (pw[2] > 0))
            {
                // implement mappoint creat
                MapPoint::Ptr mp = MapPoint::CreateNewMappoint();
                // mp->pw = current_frame_->pose_*pw;
                pw = current_frame_->Pose().inverse()*pw;
                mp->SetPos(pw);
                mp->AddObservations(feat);
                feat->mp_ = mp;
                mp->AddObservations(current_frame_->right_features_[i]);
                current_frame_->right_features_[i]->mp_  = mp;
                map_->InsertMapPoint(mp);
                map_point_num++;
            }
        }
    }
}

int Frontend::BuildInitMap()
{
    // triangulated match point in right and left img
    int map_point_num = 0;
    std::vector<MapPoint::Ptr> map_points;
    for (uint32_t i = 0; i < current_frame_->left_features_.size(); i++)
    {
        auto feat = current_frame_->left_features_[i];
        if((feat->mp_.expired()) && (current_frame_->right_features_[i]))
        {
            auto feat_r = current_frame_->right_features_[i];
            Vector3d left_pc = left_camera_->pixel2camera(
                                        Vector2d(feat->kp_pos_.pt.x, feat->kp_pos_.pt.y));
            Vector3d right_pc = right_camera_->pixel2camera(
                                        Vector2d(feat_r->kp_pos_.pt.x, feat_r->kp_pos_.pt.y));
            Vector3d pw;
            bool ret = triangulate(left_pc, right_pc, left_camera_->pose_, right_camera_->pose_, pw);
            if (ret && (pw(2) > 0))
            {
                // implement mappoint creat
                MapPoint::Ptr mp = MapPoint::CreateNewMappoint();
                mp->SetPos(pw);
                mp->AddObservations(feat);
                feat->mp_ = mp;
                mp->AddObservations(current_frame_->right_features_[i]);
                current_frame_->right_features_[i]->mp_ = mp;
                // use a vector to store?
                map_points.push_back(mp);
                map_point_num++;
            }
        }
    }

    LOG(INFO) << "success triangulate mapponit num is " << map_point_num << endl;

    if (map_point_num > initial_map_point)
    {
        for (auto map_point : map_points)
        {
            map_->InsertMapPoint(map_point);
        }
        current_frame_->SetKeyFrame();
        map_->InsertKeyframe(current_frame_);
        //for debug
        current_frame_->SetPose(Sophus::SE3d());

        // if(backend_)
        // {
        //     LOG(INFO) << "trigger for initial map opt";
        //     backend_->UpdateMap();
        //     // usleep(500*1000);
        // }
    }

    return map_point_num;
}

bool Frontend::triangulate(Vector3d point_left, Vector3d point_right, Sophus::SE3d pose_left, Sophus::SE3d pose_right, Vector3d &pw)
{
    Eigen::Matrix4d A;
    A.block<1,4>(0,0) = point_left(0)*pose_left.matrix().row(2) - pose_left.matrix().row(0);
    A.block<1,4>(1,0) = point_left(1)*pose_left.matrix().row(2) - pose_left.matrix().row(1);
    A.block<1,4>(2,0) = point_right(0)*pose_right.matrix().row(2) - pose_right.matrix().row(0);
    A.block<1,4>(3,0) = point_right(1)*pose_right.matrix().row(2) - pose_right.matrix().row(1);
    // auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto svd = A.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    pw = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).block<3,1>(0,0);
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        // the A matrix should have freedom 3 ?
        return true;
    }

    return false;
}

void Frontend::Reset()
{
    if (!have_reset_)
    {
        have_reset_ = true;
        state_ = Frontend_state::TRACK_BAD;
        // plot current, last img and it's feature point

        LOG(INFO) << "Reset not implement";
    }
}

}