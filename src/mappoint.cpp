#include "mappoint.hpp"
#include "feature.hpp"

namespace myslam {

MapPoint::MapPoint(unsigned long id, Sophus::Vector3d &pw)
{
    id_ = id;
    pw_ = pw;
}

MapPoint::Ptr MapPoint::CreateNewMappoint()
{
    static unsigned long factory_id = 0;
    MapPoint::Ptr mp(new MapPoint());
    mp->id_ = factory_id++;
    return mp;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature)
{
    std::unique_lock<std::mutex> lck(data_mutex_);
    for(auto iter = observation.begin(); iter != observation.end(); iter++) {
        if(iter->lock() == feature)
        {
            observation.erase(iter);
            feature->mp_.reset();
            observed_times_--;
            break;
        }
    }
}

}