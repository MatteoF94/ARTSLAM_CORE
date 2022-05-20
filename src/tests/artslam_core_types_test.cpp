#include "artslam_types/keyframe.h"
#include "artslam_types/types.hpp"

using namespace artslam::core::types;

int main(int argc, char** argv) {
    // testing Eigen types
    EigVector2f eig_vector_2f = {1.2, 3.4};
    EigVector2d eig_vector_2d = {1.2, 3.4};
    std::cout << "EigVector2f:\t";
    std::cout << "{" << typeid(eig_vector_2f(0)).name() << " - " << eig_vector_2f.transpose() << "}" << std::endl;
    std::cout << "EigVector2d:\t";
    std::cout << "{" << typeid(eig_vector_2d(0)).name() << " - " << eig_vector_2d.transpose() << "}" << std::endl;

    EigVector3f eig_vector_3f = {1.2, 3.4, 5.6};
    EigVector3d eig_vector_3d = {1.2, 3.4, 5.6};
    std::cout << "EigVector3f:\t";
    std::cout << "{" << typeid(eig_vector_3f(0)).name() << " - " << eig_vector_3f.transpose() << "}" << std::endl;
    std::cout << "EigVector3d:\t";
    std::cout << "{" << typeid(eig_vector_3d(0)).name() << " - " << eig_vector_3d.transpose() << "}" << std::endl;

    EigVector4f eig_vector_4f = {1.2, 3.4, 5.6, 7.8};
    EigVector4d eig_vector_4d = {1.2, 3.4, 5.6, 7.8};
    std::cout << "EigVector4f:\t";
    std::cout << "{" << typeid(eig_vector_4f(0)).name() << " - " << eig_vector_4f.transpose() << "}" << std::endl;
    std::cout << "EigVector4d:\t";
    std::cout << "{" << typeid(eig_vector_4d(0)).name() << " - " << eig_vector_4d.transpose() << "}" << std::endl;

    EigVector5f eig_vector_5f;
    eig_vector_5f << 1.2, 3.4, 5.6, 7.8, 9.1;
    EigVector5d eig_vector_5d;
    eig_vector_5d << 1.2, 3.4, 5.6, 7.8, 9.1;
    std::cout << "EigVector5f:\t";
    std::cout << "{" << typeid(eig_vector_5f(0)).name() << " - " << eig_vector_5f.transpose() << "}" << std::endl;
    std::cout << "EigVector5d:\t";
    std::cout << "{" << typeid(eig_vector_5d(0)).name() << " - " << eig_vector_5d.transpose() << "}" << std::endl;

    EigVector6f eig_vector_6f;
    eig_vector_6f << 1.2, 3.4, 5.6, 7.8, 9.1, 10.2;
    EigVector6d eig_vector_6d;
    eig_vector_6d << 1.2, 3.4, 5.6, 7.8, 9.1, 10.2;
    std::cout << "EigVector6f:\t";
    std::cout << "{" << typeid(eig_vector_6f(0)).name() << " - " << eig_vector_6f.transpose() << "}" << std::endl;
    std::cout << "EigVector6d:\t";
    std::cout << "{" << typeid(eig_vector_6d(0)).name() << " - " << eig_vector_6d.transpose() << "}" << std::endl;

    EigVectorXf eig_vector_xf(2);
    eig_vector_xf << 1.2, 3.4;
    EigVectorXd eig_vector_xd(2);
    eig_vector_xd << 1.2, 3.4;
    std::cout << "EigVectorXf:\t";
    std::cout << "{" << typeid(eig_vector_xf(0)).name() << " - " << eig_vector_xf.transpose() << "}" << std::endl;
    std::cout << "EigVectorXd:\t";
    std::cout << "{" << typeid(eig_vector_xd(0)).name() << " - " << eig_vector_xd.transpose() << "}" << std::endl;

    EigMatrix2f eig_matrix_2f = EigMatrix2f::Random();
    EigMatrix2d eig_matrix_2d = EigMatrix2d::Random();
    std::cout << "EigMatrix2f:\t";
    std::cout << "{" << typeid(eig_matrix_2f(0,0)).name() << " - \n" << eig_matrix_2f << "}" << std::endl;
    std::cout << "EigMatrix2d:\t";
    std::cout << "{" << typeid(eig_matrix_2d(0,0)).name() << " - \n" << eig_matrix_2d << "}" << std::endl;

    EigMatrix3f eig_matrix_3f = EigMatrix3f::Random();
    EigMatrix3d eig_matrix_3d = EigMatrix3d::Random();
    std::cout << "EigMatrix3f:\t";
    std::cout << "{" << typeid(eig_matrix_3f(0,0)).name() << " - \n" << eig_matrix_3f << "}" << std::endl;
    std::cout << "EigMatrix3d:\t";
    std::cout << "{" << typeid(eig_matrix_3d(0,0)).name() << " - \n" << eig_matrix_3d << "}" << std::endl;

    EigMatrix4f eig_matrix_4f = EigMatrix4f::Random();
    EigMatrix4d eig_matrix_4d = EigMatrix4d::Random();
    std::cout << "EigMatrix4f:\t";
    std::cout << "{" << typeid(eig_matrix_4f(0,0)).name() << " - \n" << eig_matrix_4f << "}" << std::endl;
    std::cout << "EigMatrix4d:\t";
    std::cout << "{" << typeid(eig_matrix_4d(0,0)).name() << " - \n" << eig_matrix_4d << "}" << std::endl;

    EigMatrix5f eig_matrix_5f = EigMatrix5f::Random();
    EigMatrix5d eig_matrix_5d = EigMatrix5d::Random();
    std::cout << "EigMatrix5f:\t";
    std::cout << "{" << typeid(eig_matrix_5f(0,0)).name() << " - \n" << eig_matrix_5f << "}" << std::endl;
    std::cout << "EigMatrix5d:\t";
    std::cout << "{" << typeid(eig_matrix_5d(0,0)).name() << " - \n" << eig_matrix_5d << "}" << std::endl;

    EigMatrix6f eig_matrix_6f = EigMatrix6f::Random();
    EigMatrix6d eig_matrix_6d = EigMatrix6d::Random();
    std::cout << "EigMatrix6f:\t";
    std::cout << "{" << typeid(eig_matrix_6f(0,0)).name() << " - \n" << eig_matrix_6f << "}" << std::endl;
    std::cout << "EigMatrix6d:\t";
    std::cout << "{" << typeid(eig_matrix_6d(0,0)).name() << " - \n" << eig_matrix_6d << "}" << std::endl;

    EigMatrixXf eig_matrix_xf = EigMatrix2f::Random();
    EigMatrixXd eig_matrix_xd = EigMatrix2d::Random();
    std::cout << "EigMatrixXf:\t";
    std::cout << "{" << typeid(eig_matrix_xf(0,0)).name() << " - \n" << eig_matrix_xf << "}" << std::endl;
    std::cout << "EigMatrixXd:\t";
    std::cout << "{" << typeid(eig_matrix_xd(0,0)).name() << " - \n" << eig_matrix_xd << "}" << std::endl;

    EigIsometry2f eig_isometry_2f = EigIsometry2f::Identity();
    EigIsometry2d eig_isometry_2d = EigIsometry2d::Identity();
    std::cout << "EigIsometry2f:\t";
    std::cout << "{" << typeid(eig_isometry_2f(0,0)).name() << " - \n" << eig_isometry_2f.matrix() << "}" << std::endl;
    std::cout << "EigIsometry2d:\t";
    std::cout << "{" << typeid(eig_isometry_2d(0,0)).name() << " - \n" << eig_isometry_2d.matrix() << "}" << std::endl;

    EigIsometry3f eig_isometry_3f = EigIsometry3f::Identity();
    EigIsometry3d eig_isometry_3d = EigIsometry3d::Identity();
    std::cout << "EigIsometry3f:\t";
    std::cout << "{" << typeid(eig_isometry_3f(0,0)).name() << " - \n" << eig_isometry_3f.matrix() << "}" << std::endl;
    std::cout << "EigIsometry3d:\t";
    std::cout << "{" << typeid(eig_isometry_3d(0,0)).name() << " - \n" << eig_isometry_3d.matrix() << "}" << std::endl;

    EigQuaternionf eig_quaternion_f = EigQuaternionf::UnitRandom();
    EigQuaterniond eig_quaternion_d = EigQuaterniond::UnitRandom();
    std::cout << "EigQuaternionf:\t";
    std::cout << "{" << typeid(eig_quaternion_f.w()).name() << " - " << eig_quaternion_f.coeffs().transpose() << "}" << std::endl;
    std::cout << "EigQuaterniond:\t";
    std::cout << "{" << typeid(eig_quaternion_d.w()).name() << " - " << eig_quaternion_d.coeffs().transpose() << "}" << std::endl;

    EigAngleAxisf eig_angle_axis_f = EigAngleAxisf::Identity();
    EigAngleAxisd eig_angle_axis_d = EigAngleAxisd::Identity();
    std::cout << "EigAngleAxisf:\t";
    std::cout << "{" << typeid(eig_angle_axis_f.angle()).name() << " - \n" << eig_angle_axis_f.toRotationMatrix() << "}" << std::endl;
    std::cout << "EigAngleAxisd:\t";
    std::cout << "{" << typeid(eig_angle_axis_d.angle()).name() << " - \n" << eig_angle_axis_d.toRotationMatrix() << "}" << std::endl;

    // testing PCL types
    Point2 point_2;
    point_2.x = 1.2;
    point_2.y = 3.4;
    std::cout << "Point2:\t\t";
    std::cout << "{" << typeid(point_2).name() << " - " << point_2.x << " " << point_2.y << "}" << std::endl;

    Point3 point_3;
    point_3.x = 1.2;
    point_3.y = 3.4;
    point_3.z = 5.6;
    std::cout << "Point3:\t\t";
    std::cout << "{" << typeid(point_3).name() << " - " << point_3.x << " " << point_3.y << " " << point_3.z << "}" << std::endl;

    Point3I point_3i;
    point_3i.x = 1.2;
    point_3i.y = 3.4;
    point_3i.z = 5.6;
    point_3i.intensity = 0.7;
    std::cout << "Point3I:\t";
    std::cout << "{" << typeid(point_3i).name() << " - " << point_3i.x << " " << point_3i.y << " " << point_3i.z << " " << point_3i.intensity << "}" << std::endl;

    Point3N point_3n;
    point_3n.x = 1.2;
    point_3n.y = 3.4;
    point_3n.z = 5.6;
    point_3n.normal_x = 0.0;
    point_3n.normal_y = 0.0;
    point_3n.normal_z = 1.0;
    std::cout << "Point3N:\t";
    std::cout << "{" << typeid(point_3n).name() << " - " << point_3n.x << " " << point_3n.y << " " << point_3n.z << " " << point_3n.normal_x << " " << point_3n.normal_y << " " << point_3n.normal_z << "}" << std::endl;

    Point3RGB point_3rgb;
    point_3rgb.x = 1.2;
    point_3rgb.y = 3.4;
    point_3rgb.z = 5.6;
    uint8_t r = 25, g = 75, b = 125;
    point_3rgb.r = r;
    point_3rgb.g = g;
    point_3rgb.b = b;
    std::cout << "Point3RGB:\t";
    std::cout << "{" << typeid(point_3rgb).name() << " - " << point_3rgb.x << " " << point_3rgb.y << " " << point_3rgb.z << " " << static_cast<int>(point_3rgb.r) << " " << static_cast<int>(point_3rgb.g) << " " << static_cast<int>(point_3rgb.b) << "}" << std::endl;

    // testing SLAM types
    // TODO

    // testing keyframe
    Keyframe keyframe(1234567891011, 150, 0.6);
    std::cout << "Keyframe:\t";
    std::cout << "{" << typeid(keyframe).name() << " - " << keyframe.timestamp_ << " " << keyframe.accumulated_distance_ << " " << keyframe.reliability_ << "}" << std::endl;

    return 0;
}