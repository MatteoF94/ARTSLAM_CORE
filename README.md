# ARTSLAM_CORE
***artslam_core*** is an open source c++ package including core elements to perform SLAM. It consists of several modules, including Input/Output, generic types (e.g. vectors, matrices, images, point clouds), utility methods and g2o graph optimization types and constraints. This package is mandatory for all the systems of the ART-SLAM family.

## Requirements
***artslam_core*** requires the following libraries:

- Eigen3
- Boost > 1.65.1
- PCL > 1.10
- OpenCV > 4.0
- g2o
- suitesparse

# Build
***artslam_core*** is built using catkin, although ROS is not mandatory:
```bash
cd catkin_ws/src
git clone https://github.com/MatteoF94/ARTSLAM_CORE.git
cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
