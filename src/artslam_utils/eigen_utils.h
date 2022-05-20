#ifndef ARTSLAM_CORE_EIGEN_UTILS_H
#define ARTSLAM_CORE_EIGEN_UTILS_H

#include <artslam_types/types_eigen.hpp>

using namespace artslam::core::types;

namespace artslam::core::utils {
    /*!
     * @brief Computes the skew-symmetric matrix for a vector of three or six elements
     * @tparam T Templated type of vectors and matrices
     * @param vector Vector to convert
     * @return The skew-symmetric matrix of the vector
     */
    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cross(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vector);

    /*!
     * @brief Computes the skew-symmetric matrix for a vector of three elements
     * @tparam T Templated type of vectors and matrices
     * @param vector Vector to convert
     * @return The skew-symmetric matrix of the vector
     */
    template <typename T>
    Eigen::Matrix<T, 3, 3> cross(const Eigen::Matrix<T, 3, 1>& vector);

    /*!
     * @brief Computes the skew-symmetric matrix for a vector of six elements
     * @tparam T Templated type of vectors and matrices
     * @param vector Vector to convert
     * @return The skew-symmetric matrix of the vector
     */
    template <typename T>
    Eigen::Matrix<T, 4, 4> cross(const Eigen::Matrix<T, 6, 1>& vector);

    /*!
     * @brief Applies the circle dot operator on a vector of four elements
     * @tparam T Templated type of vectors and matrices
     * @param vector Vector to convert
     * @return The matrix derived using the circle dot operator
     */
    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> circle_dot(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vector);

    /*!
     * @brief Ensures that all columns of a rotation matrix R are orthogonal w.r.t. each other
     * @tparam T Templated type of vectors and matrices
     * @param R Rotation matrix for which orthogonality is enforced
     */
    template <typename T>
    void ensure_orthogonality(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& R);

    /*!
     * @brief Converts the Lie Vector xi = [rho, phi]^ (6 x 1) --> SE(3) T = [R, t; 0 0 0 1] (4 x 4)
     * @tparam Type Templated type of vectors and matrices
     * @param xi Lie Vector se(3) corresponding to a translation and rotation
     * @return The corresponding SE(3) transformation matrix
     */
    template <typename Type>
    Eigen::Matrix<Type, 4, 4> se3_to_SE3(Eigen::Matrix<Type, Eigen::Dynamic, 1> xi);

    /*!
     * @brief Converts the SE(3) transformation T = [R, t; 0 0 0 1] (4 x 4) --> se(3) xi = [rho, phi]^ (6 x 1)
     * @tparam Type Templated type of vectors and matrices
     * @param T SE(3) transformation
     * @return The corresponding Lie Vector se(3)
     */
    template <typename Type>
    Eigen::Matrix<Type, Eigen::Dynamic, 1> SE3_to_se3(Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> T);

    /*!
     * @brief Converts a point from cartesian coordinates (x, y, z) to cylindrical coordinates (r, theta, z)
     * @tparam T Templated type of vectors and matrices
     * @param cartesian 3D point in cartesian homogeneous coordinates (x, y, z, 1)
     * @return The 3D point in cylindrical coordinates (r, theta, z, 1)
     */
    template <typename T>
    Eigen::Matrix<T, 4, 1> cartesian_to_cylindrical(Eigen::Matrix<T, 4, 1> cartesian);

    /*!
     * @brief Creates a random subset of unique elements, within a fixed range
     * @param maximum_value Maximum allowed value in the subset
     * @param subset_size Number of elements randomly generated
     * @return The subset of randomly generated numbers
     */
    std::vector<int> create_random_subset(int maximum_value, int subset_size);
}

#endif //ARTSLAM_CORE_EIGEN_UTILS_H
