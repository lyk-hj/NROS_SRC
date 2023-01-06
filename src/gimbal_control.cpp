#include "gimbal_control.h"

#define SHOW_MEASURE_RRECT

using namespace cv;
using namespace Eigen;

namespace robot_detection{

AngleSolve::AngleSolve()
{
    F_MAT=(Mat_<double>(3, 3) <<   1579.60532, 0.000000000000, 627.56545,
                        0.000000000000, 1579.86788, 508.65311,
                        0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(Mat_<double>(1, 5) << -0.09082  , 0.22923  , -0.00020  , 0.00013 , 0.00000);

    big_w = 0.225;
    big_h = 0.057;
    small_w = 0.135;
    small_h = 0.057;

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

Eigen::Vector3d AngleSolve::cam2imu(Vector3d &cam_pos)
{
//    std::cout<<"cam_pos:"<<cam_pos<<std::endl;
    // cam 2 imu(init),through test get, xyz-rpy
    double r=90.0;
    double p=0;
    double y=90.0;
    Vector3d euler_cam2imu = {r/180.0*CV_PI,p/180.0*CV_PI,y/180.0*CV_PI};  // xyz-rpy
    cam2imu_r = eulerAnglesToRotationMatrix(euler_cam2imu);
//    std::cout<<"cam2imu_r"<<cam2imu_r<<std::endl;


    Vector3d pos_tmp;
    pos_tmp = cam2imu_r * cam_pos;
//    pos_tmp = {cam_pos[2],-cam_pos[0],-cam_pos[1]};
//    std::cout<<"tmp_pos: "<<pos_tmp<<std::endl;

    //
    Vector3d euler_imu = {(double)ab_roll/180.0*CV_PI,(double)ab_pitch/180.0*CV_PI,(double)ab_yaw/180.0*CV_PI};  // xyz-rpy
    imu_r = eulerAnglesToRotationMatrix2(euler_imu);//here should take an experiment to make sure 1 or 2
//    std::cout<<"imu_r"<<imu_r<<std::endl;

    Vector3d imu_pos;
    imu_pos = imu_r * pos_tmp;
//    std::cout<<"imu_pos: "<<imu_pos<<std::endl;
    return imu_pos;
}

Eigen::Vector3d AngleSolve::imu2cam(Vector3d &imu_pos)
{
    Vector3d cam_pos;
    cam_pos = cam2imu_r.inverse()*imu_r.inverse()*imu_pos;
    return cam_pos;
}

Point2f AngleSolve::imu2pixel(Vector3d &imu_pos)
{
    Vector3d cam_pos = imu2cam(imu_pos);

    Point pixel_pos = cam2pixel(cam_pos);

    return pixel_pos;
}

Point2f AngleSolve::cam2pixel(Eigen::Vector3d &cam_pos)
{
    Vector3d tmp_pixel;
    Matrix3d F_vec;
    cv2eigen(F_MAT,F_vec);
    tmp_pixel = F_vec * cam_pos;
    // std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
    Point pixel_pos = {(int)(tmp_pixel[0]/tmp_pixel[2]),(int)(tmp_pixel[1]/tmp_pixel[2])};

    return pixel_pos;
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

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d &imu_Pos)
{
    //at world coordinate system
    auto y = -(double)imu_Pos(2,0);
    auto x = (double)sqrt(imu_Pos(0,0)*imu_Pos(0,0)+imu_Pos(1,0)*imu_Pos(1,0));
    double y_temp, y_actual, dy;
    double a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (double)atan2(y_temp, x);
        y_actual = BulletModel(x, bullet_speed, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabs(dy) < 0.001) {
            break;
        }
//        printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/CV_PI,y_temp,dy);
    }

    return Vector3d(imu_Pos(0,0),imu_Pos(1,0),-y_temp);
}

double AngleSolve::BulletModel(double &x, float &v, double &angle) { //x:m,v:m/s,angle:rad
    double y;
    fly_time = (double)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
    y = (double)(v * sin(angle) * fly_time - GRAVITY * fly_time * fly_time / 2);
    //printf("fly_time:%f\n",fly_time);
    return y;
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
    pu.push_back(p[3]);
    pu.push_back(p[2]);
    pu.push_back(p[1]);
    pu.push_back(p[0]);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);
    cv::cv2eigen(tvec, tv);

#ifdef SHOW_MEASURE_RRECT
    Mat rv_mat;
    Eigen::Matrix<double,3,3> rv;
    cv::Rodrigues(rvec,rv_mat);
    cv::cv2eigen(rv_mat,rv);
//    std::cout<<"rv"<<rv<<std::endl;

    Eigen::Vector3d imuPoint = {-w / 2 , -h / 2, 0.};
    Eigen::Vector3d armorPoint = rv*imuPoint + tv;//in camera coordinate
    cv::Point2f m_lu,m_ld,m_ru,m_rd;
    m_lu = cam2pixel(armorPoint);

    imuPoint = {-w / 2 , h / 2, 0.};
    armorPoint = rv*imuPoint + tv;
    m_ld = cam2pixel(armorPoint);

    imuPoint = {w / 2 , -h / 2, 0.};
    armorPoint = rv*imuPoint + tv;
    m_ru = cam2pixel(armorPoint);

    imuPoint = {w / 2 , h / 2, 0.};
    armorPoint = rv*imuPoint + tv;
    m_rd = cam2pixel(armorPoint);

    circle(_src,m_lu,3,Scalar(0,255,0),-1);
    circle(_src,m_ld,3,Scalar(255,255,0),-1);
    circle(_src,m_ru,3,Scalar(0,0,255),-1);
    circle(_src,m_rd,3,Scalar(0,255,255),-1);
    line(_src,m_lu,m_ld,Scalar(0,0,0),2);
    line(_src,m_ld,m_rd,Scalar(255,0,0),2);
    line(_src,m_rd,m_ru,Scalar(255,0,255),2);
    line(_src,m_ru,m_lu,Scalar(255,255,0),2);
//    std::cout<<"m_lu:"<<m_lu<<std::endl;
//    std::cout<<"m_ld:"<<m_ld<<std::endl;
//    std::cout<<"m_ru:"<<m_ru<<std::endl;
//    std::cout<<"m_rd:"<<m_rd<<std::endl;


#endif
//    // offset++
//   std::cout<<"distance:   "<<tv.norm()<<std::endl;

    return tv;
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
    return pos.norm() / bullet_speed;
}


Eigen::Vector3d AngleSolve::pixel2imu(Armor &armor)
{
    armor.camera_position = pnpSolve(armor.armor_pt4,armor.type);
    Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
    return imu_pos;
}

}