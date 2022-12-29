#ifndef SHSIHI_ANGLESOLVE_HPP
#define SHSIHI_ANGLESOLVE_HPP

#include "robot_status.h"
#include "opencv2/opencv.hpp"
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
    explicit AngleSolve(robot_state &Robotstate);

    void init(float r, float p, float y, float speed);

    void getAngle1(Armor &aimArmor);

    void getAngle2(Armor aimArmor);

    void getAngle(Eigen::Vector3d predicted_position);

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

    cv::Mat F_MAT;
    cv::Mat C_MAT;


    headAngle send;

    Eigen::Vector3d cam2imu(Eigen::Vector3d cam_pos);
    Eigen::Vector3d imu2cam(Eigen::Vector3d imu_pos);
    Eigen::Vector2d imu2pixel(Eigen::Vector3d imu_pos);

    Eigen::Vector3d transformPos2_World(Eigen::Vector3d &Pos);

    Eigen::Vector3d transformPos2_Camera(Eigen::Vector3d &Pos);

    Eigen::Vector3d pnpSolve(const cv::Point2f p[4], int type, int method = cv::SOLVEPNP_IPPE);

    Eigen::Vector3d gravitySolve(Eigen::Vector3d &Pos);//just consider gravity no air resistance consider

    Eigen::Vector3d airResistanceSolve(Eigen::Vector3d &Pos);//consider gravity asn air resistance

    void yawPitchSolve(Eigen::Vector3d &Pos);

    float BulletModel(float x, float v, float angle);

    double getFlyTime();
};

}
#endif //SHSIHI_ANGLESOLVE_HPP