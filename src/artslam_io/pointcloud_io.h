#ifndef ARTSLAM_CORE_POINTCLOUD_IO_H
#define ARTSLAM_CORE_POINTCLOUD_IO_H

#include "artslam_types/types_pcl.hpp"
#include <pcl/point_cloud.h>

using namespace artslam::core::types;

namespace artslam::core::io {
    class PointCloudIO {
    public:
        struct Configuration {
            bool verbose_ = false;
        };

        PointCloudIO();

        explicit PointCloudIO(const Configuration& configuration);

        pcl::PointCloud<Point3I>::Ptr read_bin_3I_pointcloud(const std::string& filename) const;

        pcl::PointCloud<Point3I>::Ptr read_pcd_3I_pointcloud(const std::string& filename) const;

        pcl::PointCloud<Point3I>::Ptr read_las_3I_pointcloud(const std::string& filename) const;

        std::vector<pcl::PointCloud<Point3I>::Ptr> read_chilean_3I_pointclouds(const std::string& filename) const;

        pcl::PointCloud<Point3I>::Ptr read_chilean_3I_pointcloud(const std::string& filename) const;

    private:
        bool verbose_ = false;
    };
}


#endif //ARTSLAM_CORE_POINTCLOUD_IO_H
