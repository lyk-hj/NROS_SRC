#include "gimbal_control.h"


using namespace cv;
using namespace Eigen;

namespace robot_detection{

AngleSolve::AngleSolve()
{
    F_MAT=(Mat_<double>(3, 3) <<   1579.60532, 0.000000000000, 627.56545,
                        0.000000000000, 1579.86788, 508.65311,
                        0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(Mat_<double>(1, 5) << -0.09082  , 0.22923  , -0.00020  , 0.00013 , 0.00000);

    big_w = 225.;
    big_h = 57.;
    small_w = 135.;
    small_h = 57.;

}

Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

Eigen::Matrix3d eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_x * R_y * R_z;
    return R;
}

Eigen::Vector3d AngleSolve::cam2imu(Vector3d cam_pos)
{
    // cam 2 imu(init),through test get, xyz-rpy
    double r=-90;
    double p=0;
    double y=-90;
//    double x_offset,y_offset,z_offset;
//    Vector3d imu_t = {x_offset,y_offset,z_offset};
    Vector3d euler_cam2imu = {r/180*CV_PI,p/180*CV_PI,y/180*CV_PI};  // xyz-rpy
    Matrix3d cam2imu_r = eulerAnglesToRotationMatrix(euler_cam2imu);
    // std::cout<<"cam2imu_r\n"<<cam2imu_r<<std::endl;

    // Rodrigues can get the same result
//    Mat euler = (Mat_<double>(3,1) << ab_roll/180*CV_PI, ab_pitch/180*CV_PI, ab_yaw/180*CV_PI);
//    Mat euler = (Mat_<double>(3,1) << (double)ab_roll,(double)ab_pitch,(double)ab_yaw);
//    Mat R;
//    Rodrigues(euler,R);
//    std::cout<<"R:  "<<R<<std::endl;

    Vector3d pos_tmp;
    pos_tmp = cam2imu_r.transpose() * cam_pos;

    pos_tmp = {cam_pos[2],-cam_pos[0],-cam_pos[1]};
    std::cout<<"tmp_pos: "<<pos_tmp<<std::endl;

    //
    Vector3d euler_imu = {(double)ab_roll/180*CV_PI,(double)ab_pitch/180*CV_PI,(double)ab_yaw/180*CV_PI};  // xyz-rpy
    Matrix3d imu_r = eulerAnglesToRotationMatrix2(euler_imu);
    std::cout<<"imu_r"<<imu_r<<std::endl;


    Vector3d imu_pos;
    imu_pos = imu_r.transpose() * pos_tmp;

//    std::cout<<"imu_pos: "<<imu_pos<<std::endl;
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2cam(Vector3d imu_pos)
{
    Vector3d euler_imu = {(double)ab_roll,(double)ab_pitch,(double)ab_yaw};  // xyz-rpy
    Matrix3d imu_r = eulerAnglesToRotationMatrix2(euler_imu);
    Vector3d cam_pos;
    cam_pos = imu_r.transpose()*imu_pos;
    return cam_pos;
}

Eigen::Vector2d AngleSolve::imu2pixel(Vector3d imu_pos)
{
    Vector3d euler_imu = {(double)ab_roll,(double)ab_pitch,(double)ab_yaw};  // xyz-rpy
    Matrix3d imu_r = eulerAnglesToRotationMatrix2(euler_imu);

    Vector3d tmp_pos;
    tmp_pos = imu_r * imu_pos;

    double r=-90/180*CV_PI;
    double p=0/180*CV_PI;
    double y=-90/180*CV_PI;
//    double x_offset,y_offset,z_offset;
//    Vector3d imu_t = {x_offset,y_offset,z_offset};
    Vector3d euler_cam2imu = {r,p,y};  // xyz-rpy
    Matrix3d cam2imu_r = eulerAnglesToRotationMatrix(euler_cam2imu);
    Vector3d cam_pos;
    cam_pos = cam2imu_r.transpose() * tmp_pos;

    // std::cout<<"cam_pos_in_fuc_imu2pixel: "<<cam_pos<<std::endl;

    Vector3d tmp_pixel;
    Matrix3d F_vec;
    cv2eigen(F_MAT,F_vec);
    tmp_pixel = F_vec * cam_pos;
    // std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
    Vector2d pixel_pos = {tmp_pixel[0]/tmp_pixel[2],tmp_pixel[1]/tmp_pixel[2]};

    return pixel_pos;
}

Eigen::Vector3d AngleSolve::transformPos2_World(Vector3d &Pos)
{
    //for debug
    ab_pitch=ab_yaw=ab_roll=0;

    Mat camera2_tuoluo = (Mat_<double>(3,1) << 0,0,0);
    Mat eular = (Mat_<double>(3,1) << -ab_pitch/180*CV_PI, -ab_yaw/180*CV_PI, -ab_roll/180*CV_PI);
    Mat rotated_mat,coordinate_mat;
    Rodrigues(camera2_tuoluo,coordinate_mat);
    Rodrigues(eular,rotated_mat);

    cv2eigen(rotated_mat,rotated_matrix);
    cv2eigen(coordinate_mat,coordinate_matrix);

    return coordinate_matrix*(rotated_matrix*Pos);
}

Eigen::Vector3d AngleSolve::transformPos2_Camera(Eigen::Vector3d &Pos)
{
    return rotated_matrix.inverse()*(coordinate_matrix.inverse()*Pos);
}

Eigen::Vector3d AngleSolve::gravitySolve(Vector3d &Pos)
{
    //at world coordinate system
    double height;

    double del_ta = pow(bullet_speed, 4) + 2 * 9.8 * Pos(1, 0) * bullet_speed * bullet_speed - 9.8 * 9.8 * Pos(2, 0) * Pos(2, 0);
    double t_2 = (9.8 * Pos(1, 0) + bullet_speed * bullet_speed - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
    height = 0.5 * 9.8 * t_2;

    return Vector3d(Pos(0,0),Pos(1,0), Pos(2,0) + height);

}

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d &Pos)
{
    //at world coordinate system
    auto y = -(float)Pos(1,0);
    auto x = (float)sqrt(Pos(0,0)*Pos(0,0)+Pos(2,0)*Pos(2,0));
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (float)atan2(y_temp, x);
        y_actual = BulletModel(x, bullet_speed, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
    }

    return Vector3d(Pos(0,0),y_temp,Pos(2,0));
}

Eigen::Vector3d AngleSolve::pnpSolve(const Point2f p[4], int type, int method)
{
    float w = type == SMALL ? small_w : big_w;
    float h = type == SMALL ? small_h : big_h;
    std::vector<cv::Point3d> ps = {
            {-w / 2 , -h / 2, 0.},
            {w / 2 , -h / 2, 0.},
            {w / 2 , h / 2, 0.},
            {-w / 2 , h / 2, 0.}
    };


    std::vector<cv::Point2f> pu;
    pu.push_back(p[2]);
    pu.push_back(p[3]);
    pu.push_back(p[0]);
    pu.push_back(p[1]);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);
    cv::cv2eigen(tvec, tv);

//    Mat R;
//    Rodrigues(rvec,R);
//
//    // offset++
   std::cout<<"distance:   "<<tv.norm()<<std::endl;

    // cam2imu(tv);

    return tv;
}

float AngleSolve::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float y;
    fly_time = (float)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
    y = (float)(v * sin(angle) * fly_time - GRAVITY * fly_time * fly_time / 2);
    //printf("fly_time:%f\n",fly_time);
    return y;
}

void AngleSolve::yawPitchSolve(Vector3d &Pos)
{
    send.yaw = atan2(Pos(0,0) ,
                     Pos(1,0)) / CV_PI*180.0 - ab_yaw;
    send.pitch = atan2(Pos(2,0) ,
                       sqrt(Pos(0,0)*Pos(0,0) + Pos(1,0)*Pos(1,0))) / CV_PI*180.0 - ab_pitch;
}

double AngleSolve::getFlyTime(Eigen::Vector3d &pos)
{
    fly_time = pos.norm() / bullet_speed;
    return fly_time * 1000;
}


Eigen::Vector3d AngleSolve::pixel2imu(Armor &armor)
{
    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type);
    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
    return imu_pos;
}

}