//
// Created by 蓬蒿浪人 on 2022/10/10.
//
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include "armor_detection.hpp"
#include "gimbal_control.h"
#include "armor_track.h"
#include "camera.h"

#define COLOR robot_detection::BLUE

using namespace cv;
using namespace robot_detection;

int main()
{
//    auto camera_warrper = new Camera;
    double time = -1;
    float data[4] = {0,0,0,0};
    robot_state robot;
    robot.updateData(data,COLOR);
    ArmorDetector autoShoot;
    autoShoot.clone(robot);
    ArmorTracker autoTrack;
    autoTrack.clone(robot);
    Skalman Singer;
    std::vector<Armor> autoTargets;
    Eigen::Vector3d predicted_position;
    Mat src;
    VideoCapture v("../sample/Record-blue.avi");
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
                double now_time = (double)getTickCount();
                if(time == -1)
                {
                    time = now_time;
                    continue;
                }
                double dt = (now_time - time) / (double)getTickFrequency();
                time = now_time;
                Eigen::Vector3d imuPos = autoTrack.pixel2imu(autoTargets[0]);
                Eigen::Matrix<double,2,1> measure(imuPos(0,0),imuPos(1,0));
                double all_time = SHOOT_DELAY + autoTrack.getFlyTime(autoTargets[0].camera_position);
                ////////////////Singer predictor//////////////////////////////
                Singer.PredictInit(dt);
                Singer.predict(false);
                Singer.correct(measure);
                Singer.PredictInit(all_time);
                Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
                predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
                ////////////////Singer predictor//////////////////////////////


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

