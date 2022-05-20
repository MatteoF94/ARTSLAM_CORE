#ifndef ARTSLAM_CORE_TYPES_SLAM_HPP
#define ARTSLAM_CORE_TYPES_SLAM_HPP

#include <string>
#include <memory>
#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include "types_eigen.hpp"
#include "types_pcl.hpp"
#include <pcl/point_cloud.h>

namespace artslam::core::types {
    // Contains general information of a sensor measurement
    typedef struct {
        uint32_t sequence_;     // sequence number of the measurement
        uint64_t timestamp_;    // time of the measurement, in nanoseconds
        std::string frame_id_;  // frame associated to the measurement, useful if working with the ROS framework
    } Header;

    // Contains general information about odometry
    typedef struct OdometryStamped3DMSG_tag{
        Header header_;                 // general information about the measurement
        EigMatrix4f odometry_;          // measured or computed odometry
        Eigen::MatrixXd covariance_;    // covariance of the measurement

        using Ptr = std::shared_ptr<struct OdometryStamped3DMSG_tag>;               // shared pointer to a OdometryStamped3D_MSG
        using ConstPtr = std::shared_ptr<const struct OdometryStamped3DMSG_tag>;    // shared pointer to a constant OdometryStamped3D_MSG
    } OdometryStamped3D_MSG;

    // Contains information about an IMU measurement; it can be filled with additional data, e.g., linear velocity
    typedef struct Imu3DMSG_tag {
        Header header_;                             // general information of the measurement
        bool has_orientation_;                      // whether the orientation is available
        Eigen::Quaterniond orientation_;            // measured orientation, expressed as a quaternion
        double orientation_covariance_[9];          // orientation covariance, 0s if unknown
        bool has_angular_velocity_;                 // whether the angular velocity is available
        EigVector3d angular_velocity_;              // measured angular velocity
        double angular_velocity_covariance_[9];     // angular velocity covariance, 0s if unknown
        bool has_linear_velocity_;                  // whether the linear velocity is available
        EigVector3d linear_velocity_;               // measured linear velocity
        double linear_velocity_covariance_[9];      // linear velocity covariance, 0s if unknown
        bool has_linear_acceleration_;              // whether the linear acceleration is available
        EigVector3d linear_acceleration_;           // measured linear acceleration
        double linear_acceleration_covariance_[9];  // linear acceleration covariance, 0s if unknown

        using Ptr = std::shared_ptr<struct Imu3DMSG_tag>;               // shared pointer to a IMU3D_MSG
        using ConstPtr = std::shared_ptr<const struct Imu3DMSG_tag>;    // shared pointer to a constant IMU3D_MSG
    } IMU3D_MSG;

    // Contains information about a GNSS measurement
    typedef struct GeoPointStampedMSG_tag {
        Header header_;             // general information of the measurement
        double latitude_ = 0.0;     // latitude in degrees, positive if above the Equator
        double longitude_ = 0.0;    // longitude in degrees, positive if east of the prime meridian
        double altitude_ = 0.0;     // altitude in meters, positive if above the WGS 84 ellipsoid

        using Ptr = std::shared_ptr<struct GeoPointStampedMSG_tag>;             // shared pointer to a GeoPointStamped_MSG
        using ConstPtr = std::shared_ptr<const struct GeoPointStampedMSG_tag>;  // shared pointer to a constant GeoPointStamped_MSG
    } GeoPointStamped_MSG;

    // Contains information about the coefficients of a detected 3D floor
    typedef struct FloorCoefficientsMSG_tag {
        Header header_;                 // general information of the measurement
        EigVector4d coefficients_;      // estimated floor coefficients

        using Ptr = std::shared_ptr<struct FloorCoefficientsMSG_tag>;       // shared pointer to a GeoPointStamped_MSG
        using ConstPtr = std::shared_ptr<const FloorCoefficientsMSG_tag>;   // shared pointer to a constant GeoPointStamped_MSG
    } FloorCoefficients_MSG;

    // Contains information about a Radar measurement
    typedef struct RadarMSG_tag {
        Header header_;                             // general information of the measurement
        std::vector<double> azimuths_;              // sensor readings, at each azimuth
        std::vector<uint64_t> azimuth_timestamps_;  // timestamps associated to each sensor reading
        std::vector<bool> azimuth_flags_;           // whether each reading is valid or not
        cv::Mat data_;                              // contains data in matrix form

        using Ptr = std::shared_ptr<struct RadarMSG_tag>;               // shared pointer to a Radar_MSG
        using ConstPtr = std::shared_ptr<const struct RadarMSG_tag>;    // shared pointer to a constant Radar_MSG
    } Radar_MSG;

    typedef struct ProcessedRadarMSG_tag {
        Header header_;
        cv::Mat cartesian_image_;
        EigMatrixXd cartesian_features_;
        std::vector<cv::KeyPoint> keypoints_;
        std::vector<uint64_t> keypoints_timestamps_;
        cv::Mat descriptors_;

        using Ptr = std::shared_ptr<struct ProcessedRadarMSG_tag>;
        using ConstPtr = std::shared_ptr<const struct ProcessedRadarMSG_tag>;
    } ProcessedRadar_MSG;
}

#endif //ARTSLAM_CORE_TYPES_SLAM_HPP
