#ifndef SHSIHI_ANGLESOLVE_HPP
#define SHSIHI_ANGLESOLVE_HPP

#include "robot_status.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include "armor_detection.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

namespace robot_detection{

struct headAngle
{
    double yaw;
    double pitch;
};
class AngleSolve : public robot_state
{
public:
    explicit AngleSolve();

//private:

    double big_w;
    double big_h;
    double small_w;
    double small_h;

    double fly_time;


    Eigen::Matrix<double,3,3> F_EGN;
    Eigen::Matrix<double,1,5> C_EGN;
    Eigen::Matrix<double,3,3> rotated_matrix;
    Eigen::Matrix<double,3,3> coordinate_matrix;
    Eigen::Matrix3d cam2imu_r;
    Eigen::Matrix3d imu_r;

    cv::Mat F_MAT;
    cv::Mat C_MAT;


    headAngle send;

    Eigen::Vector3d cam2imu(Eigen::Vector3d &cam_pos);
    Eigen::Vector3d imu2cam(Eigen::Vector3d &imu_pos);
    Eigen::Vector3d pixel2imu(Armor &armor);
    cv::Point imu2pixel(Eigen::Vector3d &imu_pos);

    Eigen::Vector3d pnpSolve(const cv::Point2f p[4], int type, int method = cv::SOLVEPNP_IPPE);

    Eigen::Vector3d gravitySolve(Eigen::Vector3d &Pos);//just consider gravity no air resistance consider

    Eigen::Vector3d airResistanceSolve(Eigen::Vector3d &imu_Pos);//consider gravity asn air resistance

    void yawPitchSolve(Eigen::Vector3d &Pos);

    double BulletModel(double &x, float &v, double &angle);

    double getFlyTime(Eigen::Vector3d &pos);
};

}
#endif //SHSIHI_ANGLESOLVE_HPP