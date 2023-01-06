#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.hpp"
#include "gimbal_control.h"
#include "armor_track.h"
#include "camera.h"

using namespace std;
using namespace cv;

int main()
{
    auto start = chrono::steady_clock::now();
//    auto camera_warrper = new Camera;
    double time = -1;
    float a[4] = {0,0,0,28};
    robot_state robot;

    ArmorDetector Detect;

    ArmorTracker Track;

    Skalman Singer;
    std::vector<Armor> Targets;
    Eigen::Vector3d predicted_position;
    Mat src;
    VideoCapture v("/home/lmx/debug.avi");
    int lost_count = 0;
//    if (camera_warrper->init())
//    {
    while(true)
    {
        if (!v.isOpened()){
            break;
        }
        v.read(src);
        if (src.empty()){
            break;
        }

        // chrono
        auto end = chrono::steady_clock::now();    //结束时间
        double duration = std::chrono::duration<double>(end - start).count();


        // detecting
        auto start2 = chrono::steady_clock::now();
        Targets = Detect.autoAim(src);
        auto end2 = chrono::steady_clock::now();    //结束时间
        double duration2 = std::chrono::duration<double>(end2 - start2).count();
        cout<<"detect_time:  "<<duration2<<endl;

        if (!Targets.empty())
        {
            std::cout<<"main get ---"<<Targets.size()<<"--- target!!!"<<std::endl;
            // std::cout<<"aim_pixel: "<<Targets[0].center<<std::endl;
//                    AS.init(gimbal_roll,gimbal_pitch,gimbal_yaw,28);
//                    Eigen::Vector3d cam_pos = AS.pnpSolve(Targets[0].armor_pt4,1,cv::SOLVEPNP_IPPE);
//                    // std::cout<<"cam_pos: "<<cam_pos<<std::endl;
//                    Eigen::Vector3d imu_pos = AS.cam2imu(cam_pos);
//                    std::cout<<"imu_pos: "<<imu_pos<<std::endl;
//                    // std::cout<<"imu_distance: "<<imu_pos.norm()<<std::endl;
//                    Eigen::Vector3d cam_pos2 = AS.imu2cam(imu_pos);
//                    // std::cout<<"cam_pos2: "<<cam_pos<<std::endl;
//                    Eigen::Vector2d pixel_pos = AS.imu2pixel(imu_pos);
//                    // std::cout<<"pixel_pos: "<<pixel_pos<<std::endl;
//                    circle(src,cv::Point2f(pixel_pos[0],pixel_pos[1]),10,cv::Scalar(0,255,255),-1);

            // AS.init(gimbal_roll, gimbal_pitch, gimbal_yaw, 28.0);
            // AS.getAngle2(Targets[0]);
        }
        else
        {
            std::cout<<"no target!!!"<<std::endl;
        }

        float dt = duration;  // 状态转移矩阵的dt2
        std::cout<<"fps: "<<1/dt<<std::endl;
        float a[4] = {0,0,0,28};

        double now_time = (double)getTickCount();
//                auto start3 = chrono::steady_clock::now();
        bool track_bool = Track.locateEnemy(src,Targets,a,dt,now_time);
//                auto end3 = chrono::steady_clock::now();    //结束时间
//                double duration3 = std::chrono::duration<double>(end3 - start3).count();
//                cout<<"track_time(ms):  "<<duration3*1000<<endl;

        if(track_bool)
        {
            std::cout<<"track!!!"<<Track.tracker_state<<"  id: "<<Track.tracking_id<<std::endl;
        }
        else
        {
            std::cout<<"loss!!!"<<std::endl;
        }

        cv::putText(src,"FPS: "+std::to_string(1/duration),cv::Point2f(0,30),cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0),1,3);
        imshow("src", src);

//                auto start5 = chrono::steady_clock::now();
        cv::waitKey(1);  // 6ms
//                auto end5 = chrono::steady_clock::now();
//                double duration_second = std::chrono::duration<double>(end5 - start5).count();
//                cout<<"gyhjklm"<<duration_second<<endl;

        // end
        start = end;
//            break;

    }
//    }


    printf("lost_count:%d\n",lost_count);


    return 0;
}

