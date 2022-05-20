#ifndef ARTSLAM_CORE_RADAR_UTILS_H
#define ARTSLAM_CORE_RADAR_UTILS_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <vector>

namespace artslam::core::utils {
    #define CTS350 0
    #define CIR204 1

    double get_azimuth_index(std::vector<double>& azimuths, double azimuth);

    void radar_polar_to_cartesian(std::vector<double> &azimuths, cv::Mat &fft_data, float radar_resolution,
                                  float cart_resolution, int cart_pixel_width, bool interpolate_crossover, cv::Mat &cart_img, int output_type,
                                  int navtech_version = CTS350);

    void polar_to_cartesian_points(std::vector<double> azimuths, Eigen::MatrixXd polar_points, float radar_resolution, Eigen::MatrixXd &cart_points);
    void polar_to_cartesian_points(std::vector<double> azimuths, std::vector<uint64_t> times, Eigen::MatrixXd polar_points, float radar_resolution, Eigen::MatrixXd &cart_points, std::vector<uint64_t> &point_times);

    void convert_to_bev(Eigen::MatrixXd &cart_points, float cart_resolution, int cart_pixel_width, std::vector<cv::Point2f> &bev_points);
    void convert_to_bev(Eigen::MatrixXd &cart_points, float cart_resolution, int cart_pixel_width, int patch_size,
                        std::vector<cv::KeyPoint> &bev_points, std::vector<uint64_t> &point_times);
    void convert_from_bev(std::vector<cv::KeyPoint> bev_points, float cart_resolution, int cart_pixel_width,
                          Eigen::MatrixXd &cart_points);
}

#endif //ARTSLAM_CORE_RADAR_UTILS_H
