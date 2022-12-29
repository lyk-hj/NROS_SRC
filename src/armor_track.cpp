#include "armor_track.h"
#include "gimbal_control.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define DRAW_CENTER_CIRCLE

namespace robot_detection {

ArmorTracker::ArmorTracker(robot_state &Robotstate) : AngleSolve(Robotstate){
    tracking_id = 0;

    lost_aim_cnt = 0;
    lost_threshold = 5;

    new_old_threshold = 10; //cm

    change_aim_cnt = 0;
    change_aim_threshold = 20;
    isChangeSameID = false;

    tracker_state = MISSING;
}

double ArmorTracker::countArmorIoU(Armor armor1, Armor armor2)
{
    double area1 = armor1.size.area();
    double area2 = armor2.size.area();

    std::vector<cv::Point2f> cross_points;
    cv::rotatedRectangleIntersection(armor1, armor2, cross_points);

    double area3 = cv::contourArea(cross_points);

    return (area3) / (area1 + area2);
}

// 选择enemy_armors，限制为同ID
void ArmorTracker::selectEnemy(std::vector<Armor> find_armors)
{

    if(find_armors.empty())
    {
        if(tracker_state == DETECTING)      // 空的&&检测状态，不参与函数
        {
            enermy_armor = Armor();
            tracking_id = 0;
            lost_aim_cnt = 0;
            change_aim_cnt = 0;
            isChangeSameID = false;
        }
        else if(find_armors.empty() && tracker_state == TRACKING)       // 空的&&跟踪状态，丢失++
        {
            lost_aim_cnt++;
            tracker_state = LOSING;
            if(lost_aim_cnt > lost_threshold)
            {
                tracker_state = DETECTING;
                tracking_id = 0;
                lost_aim_cnt = 0;
                change_aim_cnt = 0;
                isChangeSameID = false;
                enermy_armor = Armor();
            }
        }
        else if(tracker_state == LOSING)        // 空的&&丢失状态，丢失++
        {
            lost_aim_cnt++;
            if(lost_aim_cnt > lost_threshold)
            {
                tracker_state = DETECTING;
                tracking_id = 0;
                lost_aim_cnt = 0;
                change_aim_cnt = 0;
                isChangeSameID = false;
                enermy_armor = Armor();
            }
        }

        std::cout<<"Tracker State in no enemy:   "<< tracker_state<<std::endl;
        return ;
    }





    // -----------------------单目标
    if(find_armors.size() == 1)
    {

        if(tracker_state == DETECTING)      // 一个目标&&检测状态&&上一帧没有检测到ID，第一跟踪目标放入 enemy 并记录ID，计算真实坐标
        {
            // 计算真实的坐标
            find_armors[0].current_position = getRealPosition(find_armors[0]);

            // 初始化x_k1
////            KF.initial(find_armors[0].current_position);

            enermy_armor = find_armors[0];
            tracking_id = find_armors[0].id;
            tracker_state = TRACKING;
        }
        else if(tracker_state == TRACKING || tracker_state == LOSING)      // 一个目标&&跟踪状态&&与上一帧ID相同&&距离符合阈值（预测阈值和识别阈值），退出函数
        {
            // 计算真实的坐标
            find_armors[0].current_position = getRealPosition(find_armors[0]);

            // 本帧的真实坐标和上一帧的真实坐标，如果上一帧处在丢失状态呢？新旧坐标的保存以及时间的准确；记得清空上一帧的enemy再emplace_back-----------------------------------------
            double new_old_distance = (find_armors[0].current_position - enermy_armor.current_position).norm();

            // 不是检测状态，代表有enemy历史记录，相同ID且符合阈值，即可继续跟踪，否则为丢失
            if(find_armors[0].id == tracking_id || new_old_distance > new_old_threshold)
            {
                // 这一帧的放进去，转到下一帧这个就是上一帧的旧位置
                enermy_armor = find_armors[0];

                tracker_state = TRACKING;
            }
            else
            {
                lost_aim_cnt++;
                if(lost_aim_cnt > lost_threshold)
                {
                    tracker_state = DETECTING;
                    tracking_id = 0;
                    lost_aim_cnt = 0;
                    enermy_armor = Armor();
                }
                else
                {
                    tracker_state = LOSING;
                }
            }
        }

        std::cout<<"Tracker State in 1 enemy:   "<< tracker_state<<std::endl;
        return ;
    }

/*
    // 多目标

    if(find_armors.size() > 1)
    {
        if(tracker_state == DETECTING)      // 多个目标&&检测状态&&跟踪ID为0，寻找相同ID装甲板（分数高为跟踪ID），最多只有两个
        {
            // ******
            int maxGrade = find_armors[0].grade;
            tracking_id = find_armors[0].id;
            size_t index;

            for(size_t i = 1; i < find_armors.size(); ++i)
            {
                if(maxGrade < find_armors[i].grade)
                {
                    maxGrade = find_armors[i].grade;
                    tracking_id = find_armors[i].id;
                    index = i;
                }
            }

            find_armors[index].current_position = getRealPosition(find_armors[index]);
            enermy_armor = find_armors[index];

            // 初始化x_k1
////            KF.initial(find_armors[0].current_position);

            tracking_id = find_armors[0].id;
            tracker_state = TRACKING;
            return ;
        }
        else if(tracker_state == TRACKING || tracker_state == LOSING)      // 多目标&&跟踪状态，先筛选同ID的装甲板，再寻找与上一帧阈值范围内的装甲板作为这一帧需要跟踪的装甲板
        {
            bool isFind = false;
            for(size_t i = 0; i < find_armors.size(); ++i)
            {
                //寻找相同ID的装甲板，检验他是否符合阈值
                if(find_armors[i].id == tracking_id)
                {
                    //把装甲板的中心点转换到陀螺仪坐标系下
                    find_armors[i].current_position = getRealPosition(find_armors[i]);

                    double new_old_distance = (find_armors[i].current_position - enermy_armor.current_position).norm();
                    if(new_old_distance < new_old_threshold)
                    {
                        enermy_armor = find_armors[i];
                        isFind = true;
                        break;
                    }
                    else
                    {
                        change_aim_cnt++;
                        if(change_aim_cnt == change_aim_threshold)
                        {
                            // 初始化x_k1
////                            KF.initial(find_armors[0].current_position, predicted_speed);
                        }
                        if(change_aim_cnt > change_aim_threshold)
                        {
                            enermy_armor = find_armors[i];
                            isFind = true;
                            isChangeSameID =true;
                            break;
                        }
                    }
                }
            }

            if(isFind)
            {
                tracker_state = TRACKING;
            }
            else
            {
                lost_aim_cnt++;
                if(lost_aim_cnt > lost_threshold)
                {
                    tracker_state = DETECTING;
                    tracking_id = 0;
                    lost_aim_cnt = 0;
                    enermy_armor = Armor();
                }
                else
                {
                    tracker_state = LOSING;
                }
            }

        }
    }
*/
}

// 计算真实坐标
Eigen::Vector3d ArmorTracker::getRealPosition(Armor armor)
{
    Eigen::Vector3d Tvec = pnpSolve(armor.armor_pt4,armor.type, 1);

#ifdef DRAW_CENTER_CIRCLE
    //Pos(1,0) = -1 * Pos(1,0);
    Eigen::Matrix3d F;
    cv::cv2eigen(F_MAT,F);
    Eigen::Vector3d pc = Tvec;
    Eigen::Vector3d pu = F * pc / pc(2, 0);
    cv::circle(_src, {int(pu(0, 0)), int(pu(1, 0))}, 5, cv::Scalar(255,255,0), -1);
    std::cout<<"center:  ("<<pu(0, 0)<<", "<<pu(1, 0)<<", "<<pu(2,0)<<")"<<std::endl;

    //Pos(1,0) = -1 * Pos(1,0);
#endif

    Eigen::Vector3d aimInWorld = transformPos2_World(Tvec);
    return aimInWorld;
}

void ArmorTracker::getPredictedPositionAndSpeed(clock_t start_time)
{


    if(tracker_state == TRACKING)
    {
        //时间计算（数据收发延时+弹道时间+程序运行时间）,ms
        double delay = 0.01;
        double fly = getFlyTime();
        clock_t finish = clock();
        double run = (double)(finish - start_time);
        double all_time = delay + fly + run;

        if(isChangeSameID)
        {
////            KF.setP(KF.P);
        }
        Eigen::Vector3d imuPos = getRealPosition(enermy_armor);
        Eigen::Matrix<double,2,1> measure(imuPos(0,0),imuPos(1,0));
        ////////////////Singer predictor//////////////////////////////
        Singer.PredictInit(run);
        Singer.predict(false);
        Singer.correct(measure);
        Singer.PredictInit(all_time);
        Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
        predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
        ////////////////Singer predictor//////////////////////////////

        ////////////////CA predictor//////////////////////////////
        KF.predict(run,false);
        KF.correct(measure);
        predicted_result = KF.predict(all_time,true);
        predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
        ////////////////CA predictor//////////////////////////////

        predicted_position = imu2cam(predicted_position);
        getAngle(predicted_position);

    }
    else
    {

        return ;
    }
}


headAngle ArmorTracker::finalResult(cv::Mat src, std::vector<Armor> find_armors,clock_t start_time)
{
    _src = src;
    selectEnemy(find_armors);
    getPredictedPositionAndSpeed(start_time);

    //printf("-----------------%d------idid------------",enermy_armor.id);

    return send;
}

}