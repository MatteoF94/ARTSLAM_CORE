#include <iostream>

#include "artslam_io/pointcloud_io.h"
#include "artslam_io/mulran_io.h"
#include "artslam_utils/radar_utils.h"
#include "artslam_utils/eigen_utils.h"
#include <pcl/common/angles.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <chrono>
#include <pcl/visualization/cloud_viewer.h>


#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/trivial.hpp>

using namespace artslam::core::io;
using namespace artslam::core::utils;
using namespace std::chrono_literals;

class Prova {
public:
    Prova(boost::log::trivial::severity_level lev) {
        logger = boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level>(boost::log::keywords::severity = lev);
    }

    void foo(std::string string, bool skip) {
        auto start = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < 1000; i++) {
            if(!skip)
                BOOST_LOG(logger) << string;
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        std::cout << "TITTI: " << duration.count() << std::endl;
    }

    boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level> logger;
};

#include "artslam_io/kitti_reader.hpp"

int main(int argc, char** argv) {
    /*pcl::PointCloud<Point3I>::Ptr cloud(new pcl::PointCloud<Point3I>);
    mulran::read_lidar_measurement("/home/matteo/Datasets/MulRan/KAIST03/Ouster/1567410208553272076.bin", cloud);
    pcl::visualization::CloudViewer viewero ("Simple Cloud Viewer");
    viewero.showCloud (cloud);

    while (!viewero.wasStopped ())
    {
    }
    return 0;*/

    KITTI_Reader::Configuration configuration = {true, boost::log::trivial::trace};
    KITTI_Reader reader({true, boost::log::trivial::trace});
    reader.read_pointcloud("/home/matteo/miao.bin");

    Prova prova(boost::log::trivial::error), prova2(boost::log::trivial::trace);
    /*std::thread t(&Prova::foo,prova, "Prova1", true);
    std::thread t2(&Prova::foo,prova2, "Prova2", false);
    sleep(10);
    t.join();
    t2.join();*/

    return 0;

    // TODO SALVARE E TUTTO
    // pointclouds testing
    std::vector<uint64_t> timestamps;
    std::vector<double> azimuth;
    cv::Mat data;
    std::vector<bool> valid;
    mulran::read_radar_measurement("/home/matteo/Datasets/MulRan/KAIST03/polar_oxford_form/1567410201812840928.png",
                                   timestamps, azimuth, valid, data, CIR204);
    cv::Mat img2;
    radar_polar_to_cartesian(azimuth, data, 0.0432, 0.2592, 964, true, img2, CV_8UC1, CIR204);
    return 0;

    PointCloudIO::Configuration pointcloudIO_conf;
    pointcloudIO_conf.verbose_ = true;

    PointCloudIO pointcloudIO(pointcloudIO_conf);

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Generating example point clouds.\n\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    std::uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05)
    {
        for (float angle(0.0); angle <= 360.0; angle += 5.0)
        {
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * std::cos (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            std::uint32_t rgb = (static_cast<std::uint32_t>(r) << 16 |
                                 static_cast<std::uint32_t>(g) << 8 | static_cast<std::uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0)
        {
            r -= 12;
            g += 12;
        }
        else
        {
            g -= 12;
            b += 12;
        }
    }
    basic_cloud_ptr->width = basic_cloud_ptr->size ();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = point_cloud_ptr->size ();
    point_cloud_ptr->height = 1;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(point_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> (point_cloud_ptr, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        std::this_thread::sleep_for(100ms);
    }
}

