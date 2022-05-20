#include <fstream>
#include "kitti_reader.hpp"

namespace artslam::core::io {
    // Class constructor, with default parameters:
    // - configuration: configuration object of the class
    KITTI_Reader::KITTI_Reader(KITTI_Reader::Configuration configuration) : configuration_(configuration) {
        if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::info) {
            BOOST_LOG_SEV(logger_, boost::log::trivial::info) << "[KITTI_Reader] Reader created";
        }
    }

    // Reads a pointcloud, having each point defined as the tuple {x,y,z,intensity}
    // - filename: path of the file to read
    pcl::PointCloud<Point3I>::Ptr KITTI_Reader::read_pointcloud(const std::string &filename) {
        pcl::PointCloud<Point3I>::Ptr pointcloud(new pcl::PointCloud<Point3I>());

        std::fstream infile(filename.c_str(), std::ios::in | std::ios::binary);
        if (!infile.good()) {
            if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::error) {
                BOOST_LOG_SEV(logger_, boost::log::trivial::error) << "[KITTI_Reader] Could not read file: " << filename;
            }
            return pointcloud;
        }

        infile.seekg(0, std::ios::beg);
        for (int i = 0; infile.good() && !infile.eof(); i++) {
            Point3I point;
            infile.read((char *) &point.x, sizeof(float));
            infile.read((char *) &point.y, sizeof(float));
            infile.read((char *) &point.z, sizeof(float));
            infile.read((char *) &point.intensity, sizeof(float));
            pointcloud->emplace_back(point);
        }
        infile.close();

        if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::info) {
            BOOST_LOG_SEV(logger_, boost::log::trivial::info) << "[KITTI_Reader] Read pointcloud with  " << pointcloud->points.size() << " elements, from file: " << filename;
        }

        return pointcloud;
    }

    // Reads an IMU datum
    // - filename: path of the file to read
    IMU3D_MSG::Ptr KITTI_Reader::read_imu(const std::string &filename) {
        IMU3D_MSG::Ptr imu3d_msg = std::make_shared<IMU3D_MSG>();
        std::string line;
        std::ifstream infile(filename.c_str(), std::ios::in);

        if (!infile.good()) {
            if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::error) {
                BOOST_LOG_SEV(logger_, boost::log::trivial::error) << "[KITTI_Reader] Could not read file: " << filename;
            }
            return imu3d_msg;
        }

        while(std::getline(infile, line)) { // actually, it should be only one iteration
            std::istringstream line_stream(line);
            std::vector<double> values(std::istream_iterator<double>{line_stream}, std::istream_iterator<double>{});

            imu3d_msg->has_orientation_ = true;
            imu3d_msg->orientation_ = EigAngleAxisd(values[5], EigVector3d::UnitZ()) *
                                      EigAngleAxisd(values[4], EigVector3d::UnitY()) *
                                      EigAngleAxisd(values[3], EigVector3d::UnitX());
            imu3d_msg->orientation_.normalize();
            for(double& i : imu3d_msg->orientation_covariance_)
                i = 0.0;

            imu3d_msg->has_angular_velocity_ = true;
            imu3d_msg->angular_velocity_ = EigVector3d(values[17],values[18],values[19]);
            for(double& i : imu3d_msg->angular_velocity_covariance_)
                i = 0.0;

            imu3d_msg->has_linear_velocity_ = true;
            imu3d_msg->linear_velocity_ = EigVector3d(values[8],values[9],values[10]);
            for(double& i : imu3d_msg->linear_velocity_covariance_)
                i = 0.0;

            imu3d_msg->has_linear_acceleration_ = true;
            imu3d_msg->linear_acceleration_ = EigVector3d(values[11],values[12],values[13]);
            for(double& i : imu3d_msg->linear_acceleration_covariance_)
                i = 0.0;

            break;
        }
        infile.close();

        if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::info) {
            BOOST_LOG_SEV(logger_, boost::log::trivial::info) << "[KITTI_Reader] Read IMU datum from file: " << filename;
        }

        return imu3d_msg;
    }

    // Reads a GPS datum
    // - filename: path of the file to read
    GeoPointStamped_MSG::Ptr KITTI_Reader::read_gps(const std::string &filename) {
        GeoPointStamped_MSG::Ptr gps_msg = std::make_shared<GeoPointStamped_MSG>();
        std::ifstream infile(filename.c_str(), std::ios::in);
        std::string line;

        if (!infile.good()) {
            if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::error) {
                BOOST_LOG_SEV(logger_, boost::log::trivial::error) << "[KITTI_Reader] Could not read file: " << filename;
            }
            return gps_msg;
        }

        while(std::getline(infile, line)) { // actually, it should be only one iteration
            std::istringstream line_stream(line);
            std::vector<double> values(std::istream_iterator<double>{line_stream}, std::istream_iterator<double>{});

            gps_msg->latitude_ = values[0];
            gps_msg->longitude_ = values[1];
            gps_msg->altitude_ = values[2];

            break;
        }

        if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::info) {
            BOOST_LOG_SEV(logger_, boost::log::trivial::info) << "[KITTI_Reader] Read GPS datum from file: " << filename;
        }

        return gps_msg;
    }

    // Reads a list of timestamps, associated to one or multiple sensors
    // - filename: path of the file to read
    // - timestamps: vector of timestamps to fill
    void KITTI_Reader::read_timestamps(const std::string &filename, std::vector<uint64_t> &timestamps) {
        std::ifstream infile(filename.c_str(), std::ios::in);
        std::string line;

        if (!infile.good()) {
            if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::error) {
                BOOST_LOG_SEV(logger_, boost::log::trivial::error) << "[KITTI_Reader] Could not read file: " << filename;
            }
            return;
        }

        while(std::getline(infile, line)) {
            std::stringstream line_stream(line);
            std::string timestamp_string = line_stream.str();
            std::tm t = {};
            t.tm_year = std::stoi(timestamp_string.substr(0,4)) - 1900;
            t.tm_mon = std::stoi(timestamp_string.substr(5,2)) - 1;
            t.tm_mday = std::stoi(timestamp_string.substr(8,2));
            t.tm_hour = std::stoi(timestamp_string.substr(11,2));
            t.tm_min = std::stoi(timestamp_string.substr(14,2));
            t.tm_sec = std::stoi(timestamp_string.substr(17,2));
            t.tm_isdst = -1;

            static const uint64_t k_seconds_to_nanoseconds = 1e9;
            uint64_t time_since_epoch = mktime(&t);
            uint64_t timestamp = time_since_epoch * k_seconds_to_nanoseconds + std::stoi(timestamp_string.substr(20,9));
            timestamps.push_back(timestamp);
        }

        if(configuration_.verbose_ && configuration_.logging_level_ <= boost::log::trivial::info) {
            BOOST_LOG_SEV(logger_, boost::log::trivial::info) << "[KITTI_Reader] Read timestamps (" << timestamps.front() << "," << timestamps.back() << "), from file: " << filename;
        }
    }
}