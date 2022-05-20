#ifndef ARTSLAM_CORE_MULRAN_IO_H
#define ARTSLAM_CORE_MULRAN_IO_H

#include "artslam_types/types_slam.hpp"
#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>
#include <string>

using namespace artslam::core::types;

namespace artslam::core::io::mulran {
    #define CTS350 0
    #define CIR204 1

    void read_radar_measurement(const std::string& filename, std::vector<uint64_t>& timestamps, std::vector<double>& azimuths, std::vector<bool>& valid, cv::Mat& data, int navtech_version = CTS350);
    void read_radar_measurement(const std::string& filename, const Radar_MSG::Ptr& radar_msg, int navtech_version = CTS350);

    void read_lidar_measurement(const std::string& filename, const pcl::PointCloud<Point3I>::Ptr& pointcloud);
}


#endif //ARTSLAM_CORE_MULRAN_IO_H
