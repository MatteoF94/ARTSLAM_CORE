#include "eigen_utils.h"
#include <iostream>
#include <random>

namespace artslam::core::utils {
    // Computes the skew-symmetric matrix for a vector of three or six elements
    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> cross(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vector) {
        assert(vector.rows() == 3 || vector.rows() == 6);
        if (vector.rows() == 3) {
            Eigen::Matrix<T, 3, 1> vector3;
            vector3 << vector(0), vector(1), vector(2);
            return cross(vector3);
        } else {
            Eigen::Matrix<T, 6, 1> vector6;
            vector6 << vector(0), vector(1), vector(2), vector(3), vector(4), vector(5);
            return cross(vector6);
        }
    }
    template Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> cross(const Eigen::Matrix<float, Eigen::Dynamic, 1>& vector);
    template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> cross(const Eigen::Matrix<double, Eigen::Dynamic, 1>& vector);

    // Computes the skew-symmetric matrix for a vector of three elements
    template <typename T>
    Eigen::Matrix<T, 3, 3> cross(const Eigen::Matrix<T, 3, 1>& vector) {
        Eigen::Matrix<T, 3, 3> matrix = Eigen::Matrix<T, 3, 3>::Zero();
        matrix << 0, -vector(2), vector(1),
                vector(2), 0, -vector(0),
                -vector(1), vector(0), 0;
        return matrix;
    }
    template Eigen::Matrix<float, 3, 3> cross(const Eigen::Matrix<float, 3, 1>& vector);
    template Eigen::Matrix<double, 3, 3> cross(const Eigen::Matrix<double, 3, 1>& vector);

    // Computes the skew-symmetric matrix for a vector of six elements
    template <typename T>
    Eigen::Matrix<T, 4, 4> cross(const Eigen::Matrix<T, 6, 1>& vector) {
        Eigen::Matrix<T, 4, 4> matrix = Eigen::Matrix<T, 4, 4>::Zero();
        matrix << 0, -vector(5), vector(4), vector(0),
                vector(5), 0, -vector(3), vector(1),
                -vector(4), vector(3), 0, vector(2),
                0, 0, 0, 1;
        return matrix;
    }
    template Eigen::Matrix<float, 4, 4> cross(const Eigen::Matrix<float, 6, 1>& vector);
    template Eigen::Matrix<double, 4, 4> cross(const Eigen::Matrix<double, 6, 1>& vector);

    // Applies the circle dot operator on a vector of four elements
    template <typename T>
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> circle_dot(const Eigen::Matrix<T, Eigen::Dynamic, 1>& vector) {
        assert(vector.rows() == 4);
        Eigen::Matrix<T, 3, 1> rho = vector.block(0, 0, 3, 1);
        double eta = vector(3);
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Zero(4, 6);
        matrix.block(0, 0, 3, 3) = Eigen::Matrix<T, 3, 3>::Identity() * eta;
        matrix.block(0, 3, 3, 3) = -1 * cross(rho);
        return matrix;
    }
    template Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> circle_dot(const Eigen::Matrix<float, Eigen::Dynamic, 1>& vector);
    template Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> circle_dot(const Eigen::Matrix<double, Eigen::Dynamic, 1>& vector);

    // Ensures that all columns of a rotation matrix R are orthogonal w.r.t. each other
    template <typename T>
    void ensure_orthogonality(Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& R) {
        if (R.cols() == 3) {
            const Eigen::Matrix<T, 3, 1> second_column = R.block(0, 1, 3, 1).normalized();
            const Eigen::Matrix<T, 3, 1> third_column = R.block(0, 2, 3, 1).normalized();
            const Eigen::Matrix<T, 3, 1> first_column_ortho = second_column.cross(third_column);
            const Eigen::Matrix<T, 3, 1> second_column_ortho = third_column.cross(first_column_ortho);
            R.block(0, 0, 3, 1) = first_column_ortho;
            R.block(0, 1, 3, 1) = second_column_ortho;
            R.block(0, 2, 3, 1) = third_column;
        } else if (R.cols() == 2) {
            T a = (R(0, 0) + R(1, 1)) / 2;
            T b = (-R(1, 0) + R(0, 1)) / 2;
            T sum = sqrt(pow(a, 2) + pow(b, 2));
            a /= sum;
            b /= sum;
            R(0, 0) = a; R(0, 1) = b;
            R(1, 0) = -b; R(1, 1) = a;
        }
    }
    template void ensure_orthogonality(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& R);
    template void ensure_orthogonality(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& R);

    // Converts the Lie Vector xi = [rho, phi]^ (6 x 1) --> SE(3) T = [R, t; 0 0 0 1] (4 x 4)
    template <typename Type>
    Eigen::Matrix<Type, 4, 4> se3_to_SE3(Eigen::Matrix<Type, Eigen::Dynamic, 1> xi) {
        assert(xi.rows() == 6);
        Eigen::Matrix<Type, 4, 4> T = Eigen::Matrix<Type, 4, 4>::Identity();
        Eigen::Matrix<Type, 3, 1> rho = xi.block(0, 0, 3, 1);
        Eigen::Matrix<Type, 3, 1> phibar = xi.block(3, 0, 3, 1);
        double phi = phibar.norm();

        // use Rodrigues' formula to create the rotation matrix, considering the normalized vector phi
        // formulas are simplified using the relation --> w*w^ / norm(w)^2 - (w_cross / norm(w)) = I
        Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> R = Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic>::Identity(3, 3);
        if (phi != 0) {
            phibar.normalize();
            Eigen::Matrix<Type, 3, 3> I = Eigen::Matrix<Type, 3, 3>::Identity();
            R = cos(phi) * I + (1 - cos(phi)) * phibar * phibar.transpose() + sin(phi) * cross(phibar);
            ensure_orthogonality(R);
            Eigen::Matrix<Type, 3, 3> V = I * sin(phi) / phi + (1 - sin(phi) / phi) * phibar * phibar.transpose() +
                                cross(phibar) * (1 - cos(phi)) / phi;
            rho = V * rho;
        }
        T.block(0, 0, 3, 3) = R;
        T.block(0, 3, 3, 1) = rho;
        return T;
    }
    template Eigen::Matrix<float, 4, 4> se3_to_SE3(Eigen::Matrix<float, Eigen::Dynamic, 1> xi);
    template Eigen::Matrix<double, 4, 4> se3_to_SE3(Eigen::Matrix<double, Eigen::Dynamic, 1> xi);

    // Converts the SE(3) transformation T = [R, t; 0 0 0 1] (4 x 4) --> se(3) xi = [rho, phi]^ (6 x 1)
    template <typename Type>
    Eigen::Matrix<Type, Eigen::Dynamic, 1> SE3_to_se3(Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> T) {
        Eigen::Matrix<Type, Eigen::Dynamic, Eigen::Dynamic> R = T.block(0, 0, 3, 3);
        Eigen::Matrix<std::complex<Type>, Eigen::Dynamic, Eigen::Dynamic> Rr = R.template cast<std::complex<Type>>();
        Eigen::ComplexEigenSolver<Eigen::Matrix<std::complex<Type>, Eigen::Dynamic, Eigen::Dynamic>> ces;
        ces.compute(Rr);
        int idx = -1;
        Eigen::Matrix<std::complex<Type>, Eigen::Dynamic, 1> eigenvalues = ces.eigenvalues();
        Eigen::Matrix<std::complex<Type>, Eigen::Dynamic, Eigen::Dynamic> eigenvectors = ces.eigenvectors();
        for (int i = 0; i < 3; ++i) {
            if (eigenvalues(i, 0).real() != 0 && eigenvalues(i, 0).imag() == 0) {
                idx = i;
                break;
            }
        }
        assert(idx != -1);
        Eigen::Matrix<Type, Eigen::Dynamic, 1> abar = Eigen::Matrix<Type, 3, 1>::Zero();
        for (int i = 0; i < abar.rows(); ++i) {
            abar(i, 0) = eigenvectors(i, idx).real();
        }
        abar.normalize();
        double trace = 0;
        for (int i = 0; i < R.rows(); ++i) {
            trace += R(i, i);
        }
        double phi = acos((trace - 1) / 2);
        Eigen::Matrix<Type, 3, 3> I = Eigen::Matrix<Type, 3, 3>::Identity();
        Eigen::Matrix<Type, 3, 3> J = I * sin(phi) / phi + (1 - sin(phi) / phi) * abar * abar.transpose() +
                            cross(abar) * (1 - cos(phi)) / phi;
        Eigen::Matrix<Type, Eigen::Dynamic, 1> rho = J.inverse() * T.block(0, 3, 3, 1);
        Eigen::Matrix<Type, Eigen::Dynamic, 1> xi = Eigen::Matrix<Type, Eigen::Dynamic, 1>::Zero(6);
        xi.block(0, 0, 3, 1) = rho;
        xi.block(3, 0, 3, 1) = phi * abar;
        return xi;
    }
    template Eigen::Matrix<float, Eigen::Dynamic, 1> SE3_to_se3(Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> T);
    template Eigen::Matrix<double, Eigen::Dynamic, 1> SE3_to_se3(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> T);

    // Converts a point from cartesian coordinates (x, y, z) to cylindrical coordinates (r, theta, z)
    template <typename T>
    Eigen::Matrix<T, 4, 1> cartesian_to_cylindrical(Eigen::Matrix<T, 4, 1> cartesian) {
        Eigen::Matrix<T, 4, 1> cylindrical = Eigen::Matrix<T, 4, 1>::Zero();
        double x = cartesian(0);
        double y = cartesian(1);
        cylindrical(0) = sqrt(pow(x, 2) + pow(y, 2));
        cylindrical(1) = atan2(y, x);
        cylindrical(2) = cartesian(2);
        cylindrical(3) = 1;
        return cylindrical;
    }
    template Eigen::Matrix<float, 4, 1> cartesian_to_cylindrical(Eigen::Matrix<float, 4, 1> cartesian);
    template Eigen::Matrix<double, 4, 1> cartesian_to_cylindrical(Eigen::Matrix<double, 4, 1> cartesian);

    // Converts a point from cartesian coordinates (x, y, z) to cylindrical coordinates (r, theta, z)
    template <typename T>
    Eigen::Matrix<T, 4, 1> cylindrical_to_cartesian(Eigen::Matrix<T, 4, 1> cylindrical) {
        Eigen::Matrix<T, 4, 1> cartesian = Eigen::Matrix<T, 4, 1>::Zero();
        double r = cylindrical(0);
        double theta = cylindrical(1);
        cartesian(0) = r * cos(theta);
        cartesian(1) = r * sin(theta);
        cartesian(2) = cylindrical(2);
        cartesian(3) = 1;
        return cartesian;
    }
    template Eigen::Matrix<float, 4, 1> cylindrical_to_cartesian(Eigen::Matrix<float, 4, 1> cylindrical);
    template Eigen::Matrix<double, 4, 1> cylindrical_to_cartesian(Eigen::Matrix<double, 4, 1> cylindrical);

    // Creates a random subset of unique elements, within a fixed range
    std::vector<int> create_random_subset(int maximum_value, int subset_size) {
        std::vector<int> subset;

        // check for extreme situations
        if (maximum_value < 0 || subset_size < 0)
            return subset;

        if (maximum_value < subset_size)
            subset_size = maximum_value;
        subset = std::vector<int>(subset_size, -1);

        // create a random generator for a uniform distribution
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distribution;

        // randomly distribute the numbers between 0 and maximum_value
        for (uint i = 0; i < subset.size(); i++) {
            while (subset[i] < 0) {
                int idx = distribution(gen) % maximum_value;
                // check for uniqueness
                if (std::find(subset.begin(), subset.begin() + i, idx) == subset.begin() + i)
                    subset[i] = idx;
            }
        }

        return subset;
    }
}
