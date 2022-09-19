#include "camera.hpp"
#include "sophus/se3.hpp"

namespace myslam {

Camera::Camera() {

}

Sophus::Vector3d Camera::world2camera(const Sophus::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
    return pose_ * T_c_w * p_w;
}

Sophus::Vector3d Camera::camera2world(const Sophus::Vector3d &p_c, const Sophus::SE3d &T_c_w) {
    return T_c_w.inverse() * pose_inv_ * p_c;
}

Sophus::Vector2d Camera::camera2pixel(const Sophus::Vector3d &p_c) {
    return Sophus::Vector2d(
        fx_*p_c(0,0)/p_c(2,0) + cx_,
        fy_*p_c(1,0)/p_c(2,0) + cy_
    );
}

Sophus::Vector3d Camera::pixel2camera(const Sophus::Vector2d &p_p, double depth) {
    return Vector3d(
        (p_p(0,0) - cx_)*depth/fx_,
        (p_p(1,0) - cy_)*depth/fy_,
        depth
    );
}

Sophus::Vector2d Camera::world2pixel(const Sophus::Vector3d &p_w, const Sophus::SE3d &T_c_w) {
    return camera2pixel(world2camera(p_w, T_c_w));
}

Sophus::Vector3d Camera::pixel2world(const Vector2d &p_p, const Sophus::SE3d &T_c_w, double depth) {
    return camera2world(pixel2camera(p_p, depth), T_c_w);
}

}