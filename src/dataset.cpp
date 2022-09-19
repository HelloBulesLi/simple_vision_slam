#include "dataset.hpp"
#include "frame.hpp"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include "sophus/se3.hpp"
#include <glog/logging.h>

using namespace std;
using namespace Eigen;

namespace myslam {
Dataset::Dataset(const std::string& dataset_path):
        dataset_path_(dataset_path) {}

bool Dataset::Init() {
    // read camera intrinsics and extrinsics
    ifstream fin(dataset_path_ + "/calib.txt");

    if(!fin) {
        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        return false;
    }

    for (int i = 0; i < 4; i++)
    {
        char camera_name[3];
        for (int k = 0; k < 3; k++) {
            fin >> camera_name[k];
        }
        double projection_data[12];
        for (int k = 0; k < 12; k++)
        {
            fin >> projection_data[k];
        }
        Eigen::Matrix3d K;
        K << projection_data[0], projection_data[1], projection_data[2],
            projection_data[4], projection_data[5], projection_data[6],
            projection_data[8], projection_data[9], projection_data[10];
        Vector3d t;
        t << projection_data[3], projection_data[7], projection_data[11];
        t = K.inverse()*t;
        K = K*0.5;
        Camera::Ptr new_camera(new Camera(K(0,0), K(1,1), K(0,2), K(1,2),
                                         t.norm(), Sophus::SE3d(Sophus::SO3d(), t)));
        cameras_.push_back(new_camera);
        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        LOG(INFO) << "Camera " << i << " K is " << endl << K << endl;
    }

    fin.close();
    current_image_index_ = 0;
    return true;
}

Frame::Ptr Dataset::NextFrame() {
    boost::format fmt("%s/image_%d/%06d.png");
    cv::Mat image_left, image_right;

    // read images
    image_left = 
            cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
                        cv::IMREAD_GRAYSCALE);
    image_right = 
            cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
                        cv::IMREAD_GRAYSCALE);
    
    if (image_left.data == nullptr || image_right.data == nullptr) {
        LOG(WARNING) << "cannot find images at index " << current_image_index_;
        return nullptr;
    }

    double scales[] = {1.0, 0.5, 0.25, 0.125};

    cv::Mat image_left_resize,image_right_resize;
    // create pyramids

    // cv::pyrDown(image_left, image_left_resize, cv::Size2i(image_left.cols/2, image_left.rows/2));
    // cv::pyrDown(image_right, image_right_resize, cv::Size2i(image_right.cols/2, image_right.rows/2));

    cv::resize(image_left, image_left_resize, cv::Size(), 0.5, 0.5,
                cv::INTER_NEAREST);
    cv::resize(image_right, image_right_resize, cv::Size(), 0.5, 0.5,
                cv::INTER_NEAREST);

    // for debug
    // imshow("Org_left_img", image_left);
    // waitKey(0);


    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left_resize;
    new_frame->right_img_ = image_right_resize;
    current_image_index_++;
    return new_frame;
}

}