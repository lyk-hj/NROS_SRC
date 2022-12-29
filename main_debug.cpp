//
// Created by 蓬蒿浪人 on 2022/10/10.
//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.hpp"
#include "gimbal_control.h"
#include "armor_track.h"
#include "camera.h"

using namespace cv;

using namespace robot_detection;

int main()
{
//    auto camera_warrper = new Camera;
    robot_state robotState;
    robotState.enermy_color = RED;
    ArmorDetector autoShoot(robotState);
    ArmorTracker autoTrack(robotState);
    std::vector<Armor> autoTargets;
    Mat src;
    VideoCapture v("../sample/Record-red.avi");
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
            clock_t start = clock();
//            camera_warrper->read_frame_rgb(src);
            robot_detection::headAngle sendAngle;
            autoTargets = autoShoot.autoAim(src);
//            angleSolve.getAngle(autoTargets[0]);
//            string yaw = "yaw:" + convertToString(angleSolve.send.yaw);
//            string pitch = "pitch:" + convertToString(angleSolve.send.pitch);
//            string angleInformation = yaw + pitch;
//            putText(src,angleInformation,Point(0,0),FONT_HERSHEY_COMPLEX,1,Scalar(0,255,0),2);
//            sendAngle = autoTrack.finalResult(src,autoTargets, start);
            imshow("src",src);
            if (!autoTargets.empty())
            {
                printf("---------------main get target!!!---------------\n");
                sort(autoTargets.begin(),autoTargets.end(),
                     [](Armor &armor1,Armor &armor2){
                    return armor1.grade > armor2.grade;});
                Eigen::Vector3d pnpResult = autoTrack.pnpSolve(autoTargets[0].armor_pt4,autoTargets[0].type);
            }
            else
            {
                lost_count++;
                printf("----------------no target\n---------------");
//                waitKey(0);
            }
            if (waitKey(10) == 27)
            {
//                camera_warrper->~Camera();
                break;
            }
        }
//    }


    printf("lost_count:%d\n",lost_count);


    return 0;
}

