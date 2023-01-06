//opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//eigen
#include <Eigen/Dense>
//matplotlib
// #include "matplotlib.h"
//ceres
//#include "ceres/ceres.h"
//c++
#include <iostream>

#include "armor_detection.hpp"
#include "armor_track.h"
#include "camera.h"

using namespace std;

int main()
{
    double gimbal_pitch, gimbal_yaw, gimbal_roll;
    cv::Mat src;
    // chrono
    auto start = chrono::steady_clock::now();
    ArmorDetector Detect;
    std::vector<Armor> Targets;

    ArmorTracker Track;
    bool isInitial = false;

    auto camera_warper = new Camera;


    if (camera_warper->init())   // 1s
    {
        while (1)
        {
            double now_time = (double)cv::getTickCount();

            // chrono
            auto end = chrono::steady_clock::now();    //结束时间
            double duration = std::chrono::duration<double>(end - start).count();

            auto start6 = chrono::steady_clock::now();
            if (camera_warper->read_frame_rgb())   // 0.003s
            {
                src = camera_warper->src.clone();
                auto end6 = chrono::steady_clock::now();    //结束时间
                double duration6 = std::chrono::duration<double>(end6 - start6).count();
//                cout<<"789:  "<<duration6<<endl;



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

                Track.AS.init(a[2],a[0],a[1],a[3]);
//                Track.KF.initial_KF();
//                if(!Targets.empty())
//                    Eigen::Vector3d qwe = Track.AS.pnpSolve(Targets[0].armor_pt4,Targets[0].type,1);


//                auto start3 = chrono::steady_clock::now();
//                Track.AS.updateData(a);
Track.locate_target = false;
                bool track_bool = Track.locateEnemy(src,Targets,now_time);
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

                camera_warper->release_data();
            }
            else
            {
                src = cv::Mat();
            }


            // end
            start = end;
//            break;
        }
        camera_warper->~Camera();
    }
    else
    {
        printf("No camera!!\n");
    }

    return 0;
}