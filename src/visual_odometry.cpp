#include "visual_odometry.hpp"

namespace myslam {

VisualOdometry::VisualOdometry(std::string config_file_path)
{
    config_file_path_ = config_file_path;
}

bool VisualOdometry::Init()
{
    // use config file to get dataset and camera class
    file_ = cv::FileStorage(config_file_path_.c_str(), cv::FileStorage::READ);

    if (file_.isOpened() == false) {
        LOG(ERROR) << "parameter file " << config_file_path_ << " does not exist.";
        file_.release();
        return false;
    }
    
    std::string data_path = file_["dataset_dir"];
    dataset_ =  Dataset::Ptr(new Dataset(data_path));

    bool ret = dataset_->Init();
    if(!ret)
    {
        LOG(ERROR) << "dataset init failed";
        return ret;
    }

    frontend_ = Frontend::Ptr(new Frontend);
    map_ = Map::Ptr(new Map);
    backend_ = Backend::Ptr(new Backend);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetCamera(dataset_->GetCamera(0), dataset_->GetCamera(1));
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetBackend(backend_);
    backend_->SetMap(map_);
    backend_->SetCamera(dataset_->GetCamera(0), dataset_->GetCamera(1));
    viewer_->SetMap(map_);

    return true;
}

void VisualOdometry::Run() {
    while(1) {
        LOG(INFO) << "VO is running";
        if (Step() == false) {
            break;
        }
    }

    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used = 
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);

     LOG(INFO) << "VO cost time: " << time_used.count() << " secons.";
    return success;
}

}