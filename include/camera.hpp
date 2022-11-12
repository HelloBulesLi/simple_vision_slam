#pragma once

#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_H

#include <eigen3/Eigen/Core>
#include "sophus/se3.hpp"

namespace myslam {

using namespace Eigen;
using namespace std;

// Class Camera complete the camera projection and unprojection:
// 1. the transform between world coordinate frame and camera coordinate frame
// 2. the transform between image plane coordinates and normalized image plane coordinates
class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
            baseline_ = 0;
    Sophus::SE3d pose_;
    Sophus::SE3d pose_inv_;

    Camera();
    Camera(double fx, double fy, double cx, double cy, double baseline,
            const Sophus::SE3d &pose):
            fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose) {
                pose_inv_ = pose_.inverse();
            }
    Sophus::SE3d pose() const {return pose_;}
    Eigen::Matrix3d K() const {
        Matrix3d k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    Sophus::Vector3d world2camera(const Sophus::Vector3d &p_w, const Sophus::SE3d &T_c_w);
    Sophus::Vector3d camera2world(const Sophus::Vector3d &p_c, const Sophus::SE3d &T_c_w);
    Sophus::Vector2d camera2pixel(const Sophus::Vector3d &p_c);
    Sophus::Vector3d pixel2camera(const Sophus::Vector2d &p_p, double depth = 1);
    Sophus::Vector3d pixel2world(const Sophus::Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth=1);
    Sophus::Vector2d world2pixel(const Sophus::Vector3d &p_w, const Sophus::SE3d &T_c_w);
};

}


#endif
