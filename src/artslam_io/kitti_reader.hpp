#ifndef ARTSLAM_CORE_KITTI_READER_HPP
#define ARTSLAM_CORE_KITTI_READER_HPP

#include <boost/log/trivial.hpp>
#include <pcl/point_cloud.h>
#include "artslam_types/types.hpp"


using namespace artslam::core::types;

namespace artslam::core::io {
    // Class used to read the sensor measurements of the KITTI (odometry and raw) dataset
    class KITTI_Reader {
    public:
        // Defines the configuration parameters of the class
        struct Configuration {
            bool verbose_ = false;
            boost::log::trivial::severity_level logging_level_ = boost::log::trivial::info;
        };

        // Class constructor, with default parameters
        // - configuration: configuration object of the class
        explicit KITTI_Reader(Configuration configuration = {false, boost::log::trivial::warning});

        // Reads a pointcloud, having each point defined as the tuple {x,y,z,intensity}
        // - filename: path of the file to read
        pcl::PointCloud<Point3I>::Ptr read_pointcloud(const std::string& filename);

        // Reads an IMU datum
        // - filename: path of the file to read
        IMU3D_MSG::Ptr read_imu(const std::string& filename);

        // Reads a GPS datum
        // - filename: path of the file to read
        GeoPointStamped_MSG::Ptr read_gps(const std::string& filename);

        // Reads a list of timestamps, associated to one or multiple sensors
        // - filename: path of the file to read
        // - timestamps: vector of timestamps to fill
        void read_timestamps(const std::string& filename, std::vector<uint64_t>& timestamps);

    private:
        // ----------------------------------------------------------------------------------
        // ---------------------------- PARAMETERS AND VARIABLES ----------------------------
        // ----------------------------------------------------------------------------------
        Configuration configuration_;
        boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger_;
    };
}

#endif //ARTSLAM_CORE_KITTI_READER_HPP
