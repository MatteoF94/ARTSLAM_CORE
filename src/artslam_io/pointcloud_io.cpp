#include "pointcloud_io.h"
#include <iostream>
#include <fstream>
#include <iterator>
#include <pcl/io/pcd_io.h>

namespace artslam::core::io {
    PointCloudIO::PointCloudIO() : verbose_(false) {}

    PointCloudIO::PointCloudIO(const Configuration &configuration) {
        verbose_ = configuration.verbose_;
    }

    pcl::PointCloud<Point3I>::Ptr PointCloudIO::read_bin_3I_pointcloud(const std::string &filename) const {
        pcl::PointCloud<Point3I>::Ptr pointcloud(new pcl::PointCloud<Point3I>());
        std::stringstream msg;

        std::fstream infile(filename.c_str(), std::ios::in | std::ios::binary);
        if (!infile.good()) {
            msg << "[PointCloudIO] Could not read file: " << filename << "\n";
            std::cerr << msg.str();
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

        if (verbose_) {
            msg << "[PointCloudIO] Read .bin point cloud with " << pointcloud->points.size()
                << " elements (with intensity), from file: " << filename << "\n";
            std::cout << msg.str();
        }

        return pointcloud;
    }

    pcl::PointCloud<Point3I>::Ptr PointCloudIO::read_pcd_3I_pointcloud(const std::string &filename) const {
        pcl::PointCloud<Point3I>::Ptr pointcloud(new pcl::PointCloud<Point3I>());
        std::stringstream msg;

        if (pcl::io::loadPCDFile(filename, *pointcloud) == 0) {
            if (verbose_) {
                msg << "[PointCloudIO] Read .bin point cloud with " << pointcloud->points.size()
                    << " elements (with intensity), from file: " << filename << "\n";
                std::cout << msg.str();
            }
        } else {
            msg << "[PointCloudIO] Could not read file: " << filename << "\n";
            std::cerr << msg.str();
        }

        return pointcloud;
    }

    pcl::PointCloud<Point3I>::Ptr PointCloudIO::read_las_3I_pointcloud(const std::string &filename) const {
        pcl::PointCloud<Point3I>::Ptr pointcloud(new pcl::PointCloud<Point3I>());
        std::stringstream msg;

        // TODO

        return pointcloud;
    }

    std::vector<pcl::PointCloud<Point3I>::Ptr>
    PointCloudIO::read_chilean_3I_pointclouds(const std::string &filename) const {
        std::vector<pcl::PointCloud<Point3I>::Ptr> pointclouds;
        std::vector<Point3I> points;
        std::vector<double> timestamps;
        std::stringstream msg;

        std::ifstream infile(filename.c_str(), std::ios::in);
        std::string line;

        std::getline(infile, line); // the first line is a header
        while (std::getline(infile, line)) { // actually, it should be only one iteration
            std::istringstream line_stream(line);
            std::vector<double> values(std::istream_iterator<double>{line_stream}, std::istream_iterator<double>{});

            Point3I point;
            point.x = values[1];
            point.y = values[2];
            point.z = values[3];
            point.intensity = values[4];

            points.emplace_back(point);
            timestamps.emplace_back(values[0]);
        }

        double start_time = timestamps[0];
        pointclouds.emplace_back(new pcl::PointCloud<Point3I>());
        pcl::PointCloud<Point3I>::Ptr current_pointcloud = pointclouds.back();
        for (int i = 1; i < points.size(); i++) {
            current_pointcloud->push_back(points[i]);

            double curr_time = timestamps[i];
            if (curr_time - start_time > 6.0) {
                start_time = curr_time;
                pointclouds.emplace_back(new pcl::PointCloud<Point3I>());
                current_pointcloud = pointclouds.back();
                continue;
            }
        }

        return pointclouds;
    }

    pcl::PointCloud<Point3I>::Ptr PointCloudIO::read_chilean_3I_pointcloud(const std::string &filename) const {
        pcl::PointCloud<Point3I>::Ptr pointcloud(new pcl::PointCloud<Point3I>());
        std::vector<double> timestamps;
        std::stringstream msg;

        std::ifstream infile(filename.c_str(), std::ios::in);
        std::string line;

        std::getline(infile, line); // the first line is a header
        while (std::getline(infile, line)) { // actually, it should be only one iteration
            std::istringstream line_stream(line);
            std::vector<double> values(std::istream_iterator<double>{line_stream}, std::istream_iterator<double>{});

            Point3I point;
            point.x = values[1];
            point.y = values[2];
            point.z = values[3];
            point.intensity = values[4];

            timestamps.push_back(values[0]);
            pointcloud->push_back(point);
        }

        pointcloud->header.stamp = 1e9 * (timestamps.front() + timestamps.back()) / 2;

        return pointcloud;
    }
}