#include <cmath>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include "radar_utils.h"

namespace artslam::core::utils {
    void radar_polar_to_cartesian(std::vector<double> &azimuths, cv::Mat &fft_data, float radar_resolution,
                                  float cart_resolution, int cart_pixel_width, bool interpolate_crossover,
                                  cv::Mat &cart_img, int output_type,
                                  int navtech_version) {
        float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
        if (cart_pixel_width % 2 == 0)
            cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;

        cv::Mat map_x = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
        cv::Mat map_y = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

        #pragma omp parallel for collapse(2)
        for (int j = 0; j < map_y.cols; ++j) {
            for (int i = 0; i < map_y.rows; ++i) {
                map_y.at<float>(i, j) = -1 * cart_min_range + j * cart_resolution;
            }
        }

        #pragma omp parallel for collapse(2)
        for (int i = 0; i < map_x.rows; ++i) {
            for (int j = 0; j < map_x.cols; ++j) {
                map_x.at<float>(i, j) = cart_min_range - i * cart_resolution;
            }
        }
        cv::Mat range = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);
        cv::Mat angle = cv::Mat::zeros(cart_pixel_width, cart_pixel_width, CV_32F);

        double azimuth_step = azimuths[1] - azimuths[0];

        #pragma omp parallel for collapse(2) num_threads(8)
        for (int i = 0; i < range.rows; ++i) {
            for (int j = 0; j < range.cols; ++j) {
                float x = map_x.at<float>(i, j);
                float y = map_y.at<float>(i, j);
                float r = (sqrt(pow(x, 2) + pow(y, 2)) - radar_resolution / 2) / radar_resolution;
                if (r < 0)
                    r = 0;
                range.at<float>(i, j) = r;
                float theta = atan2f(y, x);
                if (theta < 0)
                    theta += 2 * M_PI;
                if (navtech_version == CIR204) {
                    angle.at<float>(i, j) = get_azimuth_index(azimuths, theta);
                } else {
                    angle.at<float>(i, j) = (theta - azimuths[0]) / azimuth_step;
                }
            }
        }

        if (interpolate_crossover) {
            cv::Mat a0 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
            cv::Mat aN_1 = cv::Mat::zeros(1, fft_data.cols, CV_32F);
            for (int j = 0; j < fft_data.cols; ++j) {
                a0.at<float>(0, j) = fft_data.at<float>(0, j);
                aN_1.at<float>(0, j) = fft_data.at<float>(fft_data.rows - 1, j);
            }
            cv::vconcat(aN_1, fft_data, fft_data);
            cv::vconcat(fft_data, a0, fft_data);
            angle = angle + 1;
        }

        cv::remap(fft_data, cart_img, range, angle, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
        if (output_type == CV_8UC1) {
            double min, max;
            cv::minMaxLoc(cart_img, &min, &max);
            cart_img.convertTo(cart_img, CV_8UC1, 255.0 / max);
        }
    }

    double get_azimuth_index(std::vector<double> &azimuths, double azimuth) {
        double mind = 1000;
        double closest = 0;
        int M = azimuths.size();
        for (int i = 0; i < azimuths.size(); ++i) {
            double d = fabs(azimuths[i] - azimuth);
            if (d < mind) {
                mind = d;
                closest = i;
            }
        }
        if (azimuths[closest] < azimuth) {
            double delta = 0;
            if (closest < M - 1)
                delta = (azimuth - azimuths[closest]) / (azimuths[closest + 1] - azimuths[closest]);
            closest += delta;
        } else if (azimuths[closest] > azimuth) {
            double delta = 0;
            if (closest > 0)
                delta = (azimuths[closest] - azimuth) / (azimuths[closest] - azimuths[closest - 1]);
            closest -= delta;
        }

        return closest;
    }

    void polar_to_cartesian_points(std::vector<double> azimuths, Eigen::MatrixXd polar_points,
                                   float radar_resolution, Eigen::MatrixXd &cart_points) {
        cart_points = polar_points;
        for (uint i = 0; i < polar_points.cols(); ++i) {
            double azimuth = azimuths[polar_points(0, i)];
            double r = polar_points(1, i) * radar_resolution + radar_resolution / 2;
            cart_points(0, i) = r * cos(azimuth);
            cart_points(1, i) = r * sin(azimuth);
        }
    }

    void polar_to_cartesian_points(std::vector<double> azimuths, std::vector<uint64_t> times, Eigen::MatrixXd polar_points,
                                   float radar_resolution, Eigen::MatrixXd &cart_points, std::vector<uint64_t> &point_times) {
        cart_points = polar_points; // polar_points: 3xN (see the function cen2019features():  targets = Eigen::MatrixXd::Ones(3, size);)
        point_times = std::vector<uint64_t>(polar_points.cols());
        for (uint i = 0; i < polar_points.cols(); ++i) {
            double azimuth = azimuths[polar_points(0, i)];
            double r = polar_points(1, i) * radar_resolution + radar_resolution / 2;
            cart_points(0, i) = r * cos(azimuth); // meter
            cart_points(1, i) = r * sin(azimuth); // meter
            point_times[i] = times[polar_points(0, i)];
        }
    }


    void convert_to_bev(Eigen::MatrixXd &cart_points, float cart_resolution, int cart_pixel_width,
                        std::vector<cv::Point2f> &bev_points) {
        float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
        if (cart_pixel_width % 2 == 0)
            cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;
        bev_points.clear();
        int j = 0;
        for (uint i = 0; i < cart_points.cols(); ++i) {
            double u = (cart_min_range + cart_points(1, i)) / cart_resolution;
            double v = (cart_min_range - cart_points(0, i)) / cart_resolution;
            if (0 < u && u < cart_pixel_width && 0 < v && v < cart_pixel_width) {
                bev_points.push_back(cv::Point2f(u, v));
                cart_points(0, j) = cart_points(0, i);
                cart_points(1, j) = cart_points(1, i);
                j++;
            }
        }
        cart_points.conservativeResize(3, bev_points.size());
    }

    void convert_to_bev(Eigen::MatrixXd &cart_points, float cart_resolution, int cart_pixel_width, int patch_size,
                        std::vector<cv::KeyPoint> &bev_points, std::vector<uint64_t> &point_times) {
        float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
        if (cart_pixel_width % 2 == 0)
            cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;
        bev_points.clear();
        int j = 0;
        for (uint i = 0; i < cart_points.cols(); ++i) {
            double u = (cart_min_range + cart_points(1, i)) / cart_resolution;
            double v = (cart_min_range - cart_points(0, i)) / cart_resolution;
            if (0 < u - patch_size && u + patch_size < cart_pixel_width && 0 < v - patch_size &&
                v + patch_size < cart_pixel_width) {
                bev_points.push_back(cv::KeyPoint(u, v, patch_size));
                point_times[j] = point_times[i];
                cart_points(0, j) = cart_points(0, i);
                cart_points(1, j) = cart_points(1, i);
                j++;
            }
        }
        point_times.resize(bev_points.size());
        cart_points.conservativeResize(3, bev_points.size());
    }

    void convert_from_bev(std::vector<cv::KeyPoint> bev_points, float cart_resolution, int cart_pixel_width,
                          Eigen::MatrixXd &cart_points) {
        cart_points = Eigen::MatrixXd::Zero(2, bev_points.size());
        float cart_min_range = (cart_pixel_width / 2) * cart_resolution;
        if (cart_pixel_width % 2 == 0)
            cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution;
        for (uint i = 0; i < bev_points.size(); ++i) {
            cart_points(0, i) = cart_min_range - cart_resolution * bev_points[i].pt.y;
            cart_points(1, i) = cart_resolution * bev_points[i].pt.x - cart_min_range;
        }
    }
}
