#include "backend.hpp"
#include "g2o_types.hpp"
#include <glog/logging.h>
#include <unistd.h>

namespace myslam {

Backend::Backend()
{
    backend_thread_ = std::thread(std::bind(&Backend::BackendLoop, this));
}

void Backend::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    LOG(INFO) << "trigger a update map";
    map_update_.notify_one();
    // log_record.open("track_result.txt", ios::app | ios::out);
}

void Backend::BackendLoop()
{
    while(1)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);
        LOG(INFO) << "wait a map update, start optimization";
        active_keyframes_ = map_->GetActiveKeyFrames();
        active_landmarks_ = map_->GetActiveMapPoints();
        auto t1 = std::chrono::steady_clock::now();
        Optimize(active_keyframes_, active_landmarks_);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used = 
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO backend Global BA cost time: " << time_used.count() << " seconds.";
    }
}

// used for only use left camera observation do global BA
/*
void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
{
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
                  g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType_Cur>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::unordered_map<unsigned long, VertexSophus *> poses_vertex;
    vector<VertexPoint *> landmarks_vertex;
    std::unordered_map<unsigned long, VertexPoint *> points_vertex;
    std::unordered_map<EdgeProjection *, Feature::Ptr> edges_and_features;

    log_record.open("track_result.txt", ios::app | ios::out);

    global_BA_cnt++;

    log_record << global_BA_cnt << " BA opt start" << std::endl;

    uint32_t index = 0;
    int min_keyframe_id = 9999;
    for (auto iter = keyframes.begin(); iter != keyframes.end(); iter++)
    {
        VertexSophus *v = new VertexSophus();
        v->setId(index);
        v->setEstimate(iter->second->Pose());
        if (iter->second->keyframe_id_ < min_keyframe_id)
        {
            min_keyframe_id = iter->second->keyframe_id_;
        }
        optimizer.addVertex(v);
        poses_vertex.insert(make_pair(iter->second->keyframe_id_,v));
        index++;
        log_record << "frame " << iter->second->id_ << "keyframe "
                    << iter->second->keyframe_id_ << " pose befor opt is: " << std::endl << iter->second->Pose().matrix() << std::endl;
    }

    if (index != 1)
    {
        poses_vertex.at(min_keyframe_id)->setFixed(true);
    }

    // record rel trans between first and second pose

    
    Sophus::SE3d rel_12 = keyframes.at(min_keyframe_id)->Pose() * keyframes.at(min_keyframe_id + 1)->Pose().inverse();
    // Sophus::SE3d rel_12 = keyframes.at(min_keyframe_id + 1)->Pose().inverse();
    // Vector3d scale_ref = keyframes.at(min_keyframe_id + 1)->Pose().translation();
    Vector3d scale_ref = rel_12.translation();

    // record relative pose trans

    // Sophus::SE3d sencond_pose = keyframes.at(min_keyframe_id + 1)->Pose();


    for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
    {
        if (iter->second->observed_times_ <=1 ) continue;
        VertexPoint *v =  new VertexPoint();
        v->setId(index);
        v->setEstimate(iter->second->Pos());
        v->setMarginalized(true);
        optimizer.addVertex(v);
        landmarks_vertex.push_back(v);
        points_vertex.insert(make_pair(iter->second->id_, v));
        index++;
    }

    // Camera::Ptr left_camera_;
    // Camera::Ptr right_camera_;
    // add edge
    double chi2_th = 5.991;
    index = 0;
    int edge_count = 0;
    Eigen::Matrix2d Information;
    Information.setIdentity();
    for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++, index++)
    {
        if (points_vertex.find(iter->second->id_) == points_vertex.end()) continue;
        for(auto feat : iter->second->observation)
        {
            auto cur_feat = feat.lock();
            if(cur_feat && (!cur_feat->is_outlier_))
            {
                auto cur_frame = cur_feat->frame_.lock();
                EdgeProjection *edge;
                if (cur_feat->is_on_left_img_ )
                {
                    edge = new EdgeProjection(left_camera_->K(), left_camera_->pose_);
                }
                else
                {
                    edge = new EdgeProjection(right_camera_->K(), right_camera_->pose_);
                }

                edge->setVertex(0, poses_vertex.at(cur_frame->keyframe_id_));
                // edge->setVertex(1, landmarks_vertex[index]);
                edge->setVertex(1, points_vertex.at(iter->second->id_));
                edge->setMeasurement(Vector2d(cur_feat->kp_pos_.pt.x,cur_feat->kp_pos_.pt.y));
                edge->setInformation(Information);
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edge->setId(edge_count);
                bool add_succ = optimizer.addEdge(edge);
                if (!add_succ)
                {
                    LOG(INFO) << "add edge failed " << endl;
                }
                edges_and_features.insert(make_pair(edge, cur_feat));
                edge_count++;
            }
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    optimizer.optimize(20);

    int inliner_cnt = 0;
    int outliner_cnt = 0;

    // decide the outlier threld
    int iter_count = 0;
    int iter_max = 4;
    while(iter_count < iter_max)
    {
        for(auto elem : edges_and_features)
        {
            if (elem.first->chi2() > chi2_th)
            {
                outliner_cnt++;
            }
            else
            {
                inliner_cnt++;
            }
        }
        double inliner_ratio = inliner_cnt /(inliner_cnt + outliner_cnt); 
        if (inliner_ratio > 0.5)
        {
            break;
        }
        else
        {
            chi2_th *= 2;
        }
        iter_count++;
    }

    for(auto elem : edges_and_features)
    {
        if (elem.first->chi2() > chi2_th)
        {
            elem.second->is_outlier_ = true;
            elem.second->mp_.lock()->RemoveObservation(elem.second);
        }
        else
        {
            elem.second->is_outlier_ = false;
        }
    }

    // use factor to correction SLAM system scale
    // for (auto iter : poses_vertex)
    // {
    //     iter
    // }

    // Sophus::SE3d sencond_opt_pose = poses_vertex.at(min_keyframe_id + 1)->estimate();

    // Sophus::SE3d correct_pose = sencond_opt_pose.inverse()*sencond_pose;
    // Sophus::SE3d corrent_pworld = sencond_pose.inverse()*sencond_opt_pose;

    Sophus::SE3d rel_12_opt = poses_vertex.at(min_keyframe_id)->estimate() * poses_vertex.at(min_keyframe_id + 1)->estimate().inverse();
    // Sophus::SE3d rel_12_opt = poses_vertex.at(min_keyframe_id + 1)->estimate().inverse();
    // Vector3d scale_after_opt = poses_vertex.at(min_keyframe_id + 1)->estimate().translation();
    Vector3d scale_after_opt = rel_12_opt.translation();


    double scale_factor = scale_ref.norm()/scale_after_opt.norm();
    // double scale_factor = 1.0;

    // for debug
    LOG(INFO) << "current outlier threld is " << chi2_th;
    LOG(INFO) << "current Gobal BA iniliner/outlier is " << inliner_cnt << '/' << outliner_cnt;

    log_record << "current outlier threld is " << chi2_th << std::endl;
    log_record << "current Gobal BA iniliner/outlier is " << inliner_cnt << '/' << outliner_cnt << std::endl;
    // set vertex estiamte to pose and landmarks
    for (auto iter: keyframes)
    {
        Sophus::SE3d cur_corrected_pose = poses_vertex[iter.first]->estimate();
        if (iter.second->keyframe_id_ != min_keyframe_id)
        {
            // cur_corrected_pose.translation() = cur_corrected_pose.translation()*scale_factor;
            Sophus::SE3d rel_pos = keyframes.at(min_keyframe_id)->Pose()*cur_corrected_pose.inverse();
            rel_pos.translation() = rel_pos.translation()*scale_factor;
            // cur_corrected_pose.translation() = cur_corrected_pose.translation()*scale_factor;
            cur_corrected_pose = rel_pos.inverse()*keyframes.at(min_keyframe_id)->Pose();
            // cur_corrected_pose = cur_corrected_pose*correct_pose;
        }
        // iter.second->SetPose(poses_vertex[iter.first]->estimate());
        iter.second->SetPose(cur_corrected_pose);
        // log_record << "frame" << iter.second->id_ << " keyframe " 
        //         << iter.second->keyframe_id_ << " pose after opt is: " << std::endl << iter.second->Pose().matrix() << std::endl;
        log_record << "frame" << iter.second->id_ << " keyframe " 
                << iter.second->keyframe_id_ << " pose after opt is: " << std::endl << cur_corrected_pose.matrix() << std::endl;
    }

    index = 0;
    for (auto iter: landmarks)
    {
        // Vector3d pw = landmarks_vertex[index]->estimate();
        if (points_vertex.find(iter.second->id_) != points_vertex.end())
        {
            Vector3d pw =  points_vertex.at(iter.second->id_)->estimate();
            Vector3d pr = keyframes.at(min_keyframe_id)->Pose() * pw;
            pr = pr * scale_factor;
            // pw =  pw*scale_factor;
            pw = keyframes.at(min_keyframe_id)->Pose().inverse()*pr;
            // Vector3d pw = corrent_pworld*points_vertex.at(iter.second->id_)->estimate();
            iter.second->SetPos(pw);
            index++;
        }
    }

    // print relpose of every frame relative to frame 0
    // Frame::Ptr ref_frame = keyframes.at(0);
    // for (auto iter: keyframes)
    // {
    //     if(iter.second == ref_frame)
    //     {
    //         continue;
    //     }
    //     // log_record << "frame" << iter.second->id_ << " keyframe " 
    //     //         << iter.second->keyframe_id_ << " rel pose to frame 0 after opt is: " << std::endl << 
    //     //         ref_frame->Pose().matrix()*iter.second->Pose().matrix().inverse() << std::endl;
    //     log_record << "frame" << iter.second->id_ << " keyframe " 
    //             << iter.second->keyframe_id_ << " rel pose to frame 0 after opt is: " << std::endl << 
    //             iter.second->Pose().matrix()*ref_frame->Pose().matrix().inverse() << std::endl;
    // }

    log_record.close();
    LOG(INFO) << "a global BA finish ";
}
*/

void Backend::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
{
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
                  g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType_Cur>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::unordered_map<unsigned long, VertexSophus *> poses_vertex;
    vector<VertexPoint *> landmarks_vertex;
    std::unordered_map<unsigned long, VertexPoint *> points_vertex;
    std::unordered_map<EdgeProjection *, Feature::Ptr> edges_and_features;

    log_record.open("track_result.txt", ios::app | ios::out);

    global_BA_cnt++;

    log_record << global_BA_cnt << " BA opt start" << std::endl;

    uint32_t index = 0;
    int min_keyframe_id = 9999;
    for (auto iter = keyframes.begin(); iter != keyframes.end(); iter++)
    {
        VertexSophus *v = new VertexSophus();
        v->setId(index);
        v->setEstimate(iter->second->Pose());
        if (iter->second->keyframe_id_ < min_keyframe_id)
        {
            min_keyframe_id = iter->second->keyframe_id_;
        }
        optimizer.addVertex(v);
        poses_vertex.insert(make_pair(iter->second->keyframe_id_,v));
        index++;
        log_record << "frame " << iter->second->id_ << "keyframe "
                    << iter->second->keyframe_id_ << " pose befor opt is: " << std::endl << iter->second->Pose().matrix() << std::endl;
    }

    if (keyframes.size() > 1)
    {
        poses_vertex.at(min_keyframe_id)->setFixed(true);
    }


    for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++)
    {
        VertexPoint *v =  new VertexPoint();
        v->setId(index);
        v->setEstimate(iter->second->Pos());
        v->setMarginalized(true);
        optimizer.addVertex(v);
        landmarks_vertex.push_back(v);
        points_vertex.insert(make_pair(iter->second->id_, v));
        index++;
    }

    // Camera::Ptr left_camera_;
    // Camera::Ptr right_camera_;
    // add edge
    double chi2_th = 5.991;
    index = 0;
    int edge_count = 0;
    Eigen::Matrix2d Information;
    Information.setIdentity();
    for (auto iter = landmarks.begin(); iter != landmarks.end(); iter++, index++)
    {
        for(auto feat : iter->second->observation)
        {
            auto cur_feat = feat.lock();
            if(cur_feat && (!cur_feat->is_outlier_))
            {
                auto cur_frame = cur_feat->frame_.lock();
                EdgeProjection *edge;
                if (cur_feat->is_on_left_img_ )
                {
                    edge = new EdgeProjection(left_camera_->K(), left_camera_->pose_);
                }
                else
                {
                    edge = new EdgeProjection(right_camera_->K(), right_camera_->pose_);
                }

                edge->setVertex(0, poses_vertex.at(cur_frame->keyframe_id_));
                edge->setVertex(1, points_vertex.at(iter->second->id_));
                edge->setMeasurement(Vector2d(cur_feat->kp_pos_.pt.x,cur_feat->kp_pos_.pt.y));
                edge->setInformation(Information);
                auto rk = new g2o::RobustKernelHuber();
                rk->setDelta(chi2_th);
                edge->setRobustKernel(rk);
                edge->setId(edge_count);
                bool add_succ = optimizer.addEdge(edge);
                if (!add_succ)
                {
                    LOG(INFO) << "add edge failed " << endl;
                }
                edges_and_features.insert(make_pair(edge, cur_feat));
                edge_count++;
            }
        }
    }

    // do optimization and eliminate the outliers
    optimizer.initializeOptimization();
    // optimizer.optimize(20);
    optimizer.optimize(10);

    int inliner_cnt = 0;
    int outliner_cnt = 0;

    // decide the outlier threld
    int iter_count = 0;
    int iter_max = 4;
    while(iter_count < iter_max)
    {
        for(auto elem : edges_and_features)
        {
            if (elem.first->chi2() > chi2_th)
            {
                outliner_cnt++;
            }
            else
            {
                inliner_cnt++;
            }
        }
        double inliner_ratio = inliner_cnt /(inliner_cnt + outliner_cnt); 
        if (inliner_ratio > 0.5)
        {
            break;
        }
        else
        {
            chi2_th *= 2;
        }
        iter_count++;
    }

    for(auto elem : edges_and_features)
    {
        if (elem.first->chi2() > chi2_th)
        {
            elem.second->is_outlier_ = true;
            elem.second->mp_.lock()->RemoveObservation(elem.second);
        }
        else
        {
            elem.second->is_outlier_ = false;
        }
    }

    // for debug
    LOG(INFO) << "current outlier threld is " << chi2_th;
    LOG(INFO) << "current Gobal BA iniliner/outlier is " << inliner_cnt << '/' << outliner_cnt;

    log_record << "current outlier threld is " << chi2_th << std::endl;
    log_record << "current Gobal BA iniliner/outlier is " << inliner_cnt << '/' << outliner_cnt << std::endl;
    // set vertex estiamte to pose and landmarks
    for (auto iter: keyframes)
    {
        iter.second->SetPose(poses_vertex[iter.first]->estimate());
        log_record << "frame" << iter.second->id_ << " keyframe " 
                << iter.second->keyframe_id_ << " pose after opt is: " << std::endl << iter.second->Pose().matrix() << std::endl;
    }

    index = 0;
    for (auto iter: landmarks)
    {
        Vector3d pw =  points_vertex.at(iter.second->id_)->estimate();
        iter.second->SetPos(pw);
    }

    log_record.close();
    LOG(INFO) << "a global BA finish ";
}

}