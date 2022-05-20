#include "mulran_io.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <iostream>
#include <fstream>

namespace artslam::core::io {
    void mulran::read_radar_measurement(const std::string &filename, std::vector<uint64_t> &timestamps,
                                        std::vector<double> &azimuths, std::vector<bool> &valid, cv::Mat &data,
                                        int navtech_version) {
        int encoder_size = 5600;

        cv::Mat polar_image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        int rows = polar_image.rows;
        //cv::imshow("Polar image", polar_image);
        //cv::waitKey(0);

        timestamps = std::vector<uint64_t>(rows, 0);
        azimuths = std::vector<double>(rows, 0);
        valid = std::vector<bool>(rows, true);

        int range_bins = 3768;
        if (navtech_version == CIR204)
            range_bins = 3360;
        data = cv::Mat::zeros(rows, range_bins, CV_32F);

#pragma omp parallel
        for (int i = 0; i < rows; i++) {
            uchar *row_ptr = polar_image.ptr<uchar>(i);
            timestamps[i] = *((uint64_t *) (row_ptr));
            azimuths[i] = *((uint16_t *) (row_ptr + 8)) * 2 * M_PI / double(encoder_size);
            valid[i] = row_ptr[10] == 255;
            for (int j = 42; j < range_bins; j++) {
                data.at<float>(i, j) = (float) *(row_ptr + 11 + j) / 255.0;
            }
        }

        std::vector<std::string> parts_path, parts_name;
        boost::split(parts_path, filename, boost::is_any_of("/"));
        std::string name = parts_path.back();
        boost::split(parts_name, name, boost::is_any_of("."));
        uint64_t timestamp = std::stoll(parts_name[0]);
        std::cout << "STAMP " << timestamp << "\n";
    }

    void
    mulran::read_radar_measurement(const std::string &filename, const Radar_MSG::Ptr &radar_msg, int navtech_version) {
        int encoder_size = 5600;

        cv::Mat polar_image = cv::imread(filename, cv::IMREAD_GRAYSCALE);
        int rows = polar_image.rows;
        //cv::imshow("Polar image", polar_image);
        //cv::waitKey(0);

        radar_msg->azimuth_timestamps_ = std::vector<uint64_t>(rows, 0);
        radar_msg->azimuths_ = std::vector<double>(rows, 0);
        radar_msg->azimuth_flags_ = std::vector<bool>(rows, true);

        int range_bins = 3768;
        if (navtech_version == CIR204)
            range_bins = 3360;
        radar_msg->data_ = cv::Mat::zeros(rows, range_bins, CV_32F);

        #pragma omp parallel
        for (int i = 0; i < rows; i++) {
            uchar *row_ptr = polar_image.ptr<uchar>(i);
            radar_msg->azimuth_timestamps_[i] = *((uint64_t *) (row_ptr));
            radar_msg->azimuths_[i] = *((uint16_t *) (row_ptr + 8)) * 2 * M_PI / double(encoder_size);
            radar_msg->azimuth_flags_[i] = row_ptr[10] == 255;
            for (int j = 42; j < range_bins; j++) {
                radar_msg->data_.at<float>(i, j) = (float) *(row_ptr + 11 + j) / 255.0;
            }
        }

        std::vector<std::string> parts_path, parts_name;
        boost::split(parts_path, filename, boost::is_any_of("/"));
        std::string name = parts_path.back();
        boost::split(parts_name, name, boost::is_any_of("."));
        uint64_t timestamp = std::stoll(parts_name[0]);
        radar_msg->header_.timestamp_ = timestamp;
    }

    void mulran::read_lidar_measurement(const std::string &filename, const pcl::PointCloud<Point3I>::Ptr &pointcloud) {
        // TODO also timestamps can be read
        std::fstream infile(filename.c_str(), std::ios::in | std::ios::binary);
        if(!infile.good()) {
           std::cout << "[PointCloudIO] Could not read file: " << filename << "\n";
            return;
        }

        infile.seekg(0, std::ios::beg);
        for(int i = 0; infile.good() && !infile.eof(); i++) {
            Point3I point;
            infile.read((char *) &point.x, sizeof(float));
            infile.read((char *) &point.y, sizeof(float));
            infile.read((char *) &point.z, sizeof(float));
            infile.read((char *) &point.intensity, sizeof(float));
            pointcloud->emplace_back(point);
        }
        infile.close();

        std::vector<std::string> parts_path, parts_name;
        boost::split(parts_path, filename, boost::is_any_of("/"));
        std::string name = parts_path.back();
        boost::split(parts_name, name, boost::is_any_of("."));
        uint64_t timestamp = std::stoll(parts_name[0]);
        pointcloud->header.stamp = timestamp;
    }
}


