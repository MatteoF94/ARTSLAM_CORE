#ifndef ARTSLAM_CORE_TYPES_EIGEN_HPP
#define ARTSLAM_CORE_TYPES_EIGEN_HPP

#include <Eigen/Dense>

namespace artslam::core::types {
    typedef Eigen::Vector2f EigVector2f;            // 2 dimensional vector of floats
    typedef Eigen::Vector2d EigVector2d;            // 2 dimensional vector of doubles
    typedef Eigen::Vector3f EigVector3f;            // 3 dimensional vector of floats
    typedef Eigen::Vector3d EigVector3d;            // 3 dimensional vector of doubles
    typedef Eigen::Vector4f EigVector4f;            // 4 dimensional vector of floats
    typedef Eigen::Vector4d EigVector4d;            // 4 dimensional vector of doubles
    typedef Eigen::Matrix<float,5,1> EigVector5f;   // 5 dimensional vector of floats
    typedef Eigen::Matrix<double,5,1> EigVector5d;  // 5 dimensional vector of doubles
    typedef Eigen::Matrix<float,6,1> EigVector6f;   // 6 dimensional vector of floats
    typedef Eigen::Matrix<double,6,1> EigVector6d;  // 6 dimensional vector of doubles

    typedef Eigen::VectorXf EigVectorXf;            // vector of floats of dynamic size
    typedef Eigen::VectorXd EigVectorXd;            // vector of doubles of dynamic size

    typedef Eigen::Matrix2f EigMatrix2f;            // 2x2 matrix of floats
    typedef Eigen::Matrix2d EigMatrix2d;            // 2x2 matrix of doubles
    typedef Eigen::Matrix3f EigMatrix3f;            // 3x3 matrix of floats
    typedef Eigen::Matrix3d EigMatrix3d;            // 3x3 matrix of doubles
    typedef Eigen::Matrix4f EigMatrix4f;            // 4x4 matrix of floats
    typedef Eigen::Matrix4d EigMatrix4d;            // 4x4 matrix of doubles
    typedef Eigen::Matrix<float,5,5> EigMatrix5f;   // 5x5 matrix of floats
    typedef Eigen::Matrix<double,5,5> EigMatrix5d;  // 5x5 matrix of doubles
    typedef Eigen::Matrix<float,6,6> EigMatrix6f;   // 6x6 matrix of floats
    typedef Eigen::Matrix<double,6,6> EigMatrix6d;  // 5x5 matrix of doubles

    typedef Eigen::MatrixXf EigMatrixXf;            // matrix of floats of dynamic size
    typedef Eigen::MatrixXd EigMatrixXd;            // matrix of doubles of dynamic size

    typedef Eigen::Isometry2f EigIsometry2f;        // isometry associated to a 2D motion, floats
    typedef Eigen::Isometry2d EigIsometry2d;        // isometry associated to a 2D motion, doubles
    typedef Eigen::Isometry3f EigIsometry3f;        // isometry associated to a 3D motion, floats
    typedef Eigen::Isometry3d EigIsometry3d;        // isometry associated to a 3D motion, doubles

    typedef Eigen::Quaternionf EigQuaternionf;      // rotation expressed as a quaternion of floats
    typedef Eigen::Quaterniond EigQuaterniond;      // rotation expressed as a quaternion of doubles

    typedef Eigen::AngleAxisf EigAngleAxisf;        // rotation around a generic axis, in floats
    typedef Eigen::AngleAxisd EigAngleAxisd;        // rotation around a generic axis, in doubles
}

#endif //ARTSLAM_CORE_TYPES_EIGEN_HPP
