#ifndef MYSLAM_DATASET_H
#define MSYLAM_DATASET_H
#include "frame.hpp"
#include "camera.hpp"
#include <map>


namespace myslam {
/**
 * @brief dataset read
 * 
 */
using namespace std;

// Class Dataset finish the Camera init, read the Kitti Dataset from set path
class Dataset {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);

    /// init, and return i success
    bool Init();

    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

private:
    std::string dataset_path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
};

}

#endif