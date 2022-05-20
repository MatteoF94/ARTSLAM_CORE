#ifndef ARTSLAM_CORE_TYPES_PCL_HPP
#define ARTSLAM_CORE_TYPES_PCL_HPP

#include <pcl/point_types.h>

namespace artslam::core::types {
    typedef pcl::PointXY Point2;        // 2D point
    typedef pcl::PointXYZ Point3;       // 3D point
    typedef pcl::PointXYZI Point3I;     // 3D point with additional information about the intensity
    typedef pcl::PointNormal Point3N;   // 3D point with additional information about the normal
    typedef pcl::PointXYZRGB Point3RGB; // 3D point with additional information about the color
}

#endif //ARTSLAM_CORE_TYPES_PCL_HPP
