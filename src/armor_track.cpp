#include "armor_track.h"
#include "gimbal_control.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//#define DRAW_MATCH_ARMOR

namespace robot_detection {

ArmorTracker::ArmorTracker()
{
    locate_target = false;
    enemy_armor = Armor();

    tracker_state = MISSING;
    tracking_id = 0;

    find_aim_cnt = 0;
    find_threshold = 10;

    lost_aim_cnt = 0;
    lost_threshold = 20;

    new_old_threshold = 0.35; //m

    isChangeSameID = false;
}

void ArmorTracker::reset()
{
    t = -1;
    Singer.Reset();
    locate_target = false;
    enemy_armor = Armor();
    tracker_state = MISSING;
    tracking_id = 0;
    find_aim_cnt = 0;
    lost_aim_cnt = 0;
}

// 初始化，选择最优装甲板，设置卡尔曼的F和x_1
bool ArmorTracker::initial(std::vector<Armor> &find_armors)
{
    if(find_armors.empty())
    {
        return false;
    }

    sort(find_armors.begin(),find_armors.end(),
      [](Armor &armor1,Armor &armor2){
         return armor1.grade > armor2.grade;});

    // select enemy
    enemy_armor = find_armors[0];
    tracker_state = DETECTING;
    tracking_id = enemy_armor.id;

    enemy_armor.imu_position = AS.pixel2imu(enemy_armor);
    KF.setXPost(enemy_armor.imu_position);
    Singer.setXpos(enemy_armor.imu_position.head(2));
//    std::cout<<"track_initial"<<std::endl;
    return true;
}

// dt是两帧之间时间间隔, 跟得住目标
bool ArmorTracker::selectEnemy2(std::vector<Armor> &find_armors, double dt)
{

    predicted_enemy = KF.predict(dt,false);

    Armor matched_armor;
    bool matched = false;

    if(!find_armors.empty())
    {
        double min_position_diff = DBL_MAX;
        for(auto & armor : find_armors)
        {
            armor.imu_position= AS.pixel2imu(armor);
            Eigen::Vector3d pre = predicted_enemy.head(3);
            double position_diff = (pre - armor.imu_position).norm();
            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                matched_armor = armor;
            }
        }
        std::cout<<"min_position_diff:"<<min_position_diff<<std::endl;
        if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id) {
            matched = true;
            Eigen::Vector3d position_vec = AS.pixel2imu(matched_armor);
            predicted_enemy = KF.correct(position_vec);
        }
        else
        {
            // 本帧内是否有相同ID
            for (auto & armor : find_armors) {
                if (armor.id == tracking_id) {
                    matched = true;
                    armor.imu_position = AS.pixel2imu(armor);
                    KF.reset();
                    Eigen::Matrix<double,6,1> pos_speed;
                    pos_speed << armor.imu_position, predicted_enemy.tail(3);
                    KF.setPosAndSpeed(pos_speed);
                    predicted_enemy = pos_speed;
                    matched_armor = armor;
                    break;
                }
            }
        }
    }
    if (matched)
    {
#ifdef DRAW_MATCH_ARMOR
        cv::Mat m_a = _src.clone();
        cv::circle(m_a,matched_armor.center,matched_armor.size.width/10,cv::Scalar(255,255,0),-1);
        Eigen::Vector3d predicted_track;
        predicted_track << predicted_enemy.head(3);
        cv::Point2f p = imu2pixel(predicted_track);
        cv::circle(m_a,p,matched_armor.size.width/15,cv::Scalar(0,0,255),-1);
        cv::imshow("DRAW_MATCH_ARMOR",m_a);
#endif
        enemy_armor = matched_armor;
    }

    if (tracker_state == DETECTING) {
        // DETECTING
        if (matched) {
            find_aim_cnt++;
            if (find_aim_cnt > find_threshold) {
                find_aim_cnt = 0;
                tracker_state = TRACKING;
            }
        } else {
            find_aim_cnt = 0;
            tracker_state = MISSING;
        }

    } else if (tracker_state == TRACKING) {
        // TRACKING
        if (!matched) {
            tracker_state = LOSING;
            lost_aim_cnt++;
        }

    } else if (tracker_state == LOSING) {
        if (!matched) {
            lost_aim_cnt++;
            if (lost_aim_cnt > lost_threshold) {
                lost_aim_cnt = 0;
                tracker_state = MISSING;
            }
        } else {
            tracker_state = TRACKING;
            lost_aim_cnt = 0;
        }
    }

    if (tracker_state == MISSING)
    {
        reset();
        return false;
    }
    KF.setPosAndSpeed(matched_armor.imu_position,predicted_enemy.tail(3));

    if(tracker_state == LOSING)
    {
        enemy_armor.imu_position = predicted_enemy.head(3);
        KF.setPosAndSpeed(enemy_armor.imu_position,predicted_enemy.tail(3));
    }

    return true;
}

// 对处于跟踪和正在丢失状态时做 预测，引入各种时间
bool ArmorTracker::estimateEnemy(double dt)
{
    cv::putText(AS._src, std::to_string(tracker_state),cv::Point(50,100),1,5,cv::Scalar(0,0,255),3);
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        // enemy_armor get real information to predicted by singer
//        enemy_armor.imu_position = pixel2imu(enemy_armor);
        circle(AS._src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
        Eigen::Matrix<double,2,1> measure(enemy_armor.imu_position(0,0),enemy_armor.imu_position(1,0));
//        std::cout<<"measurex:"<<measure[0]<<std::endl;
//        std::cout<<"measurey:"<<measure[1]<<std::endl;
        double all_time = SHOOT_DELAY + AS.getFlyTime(enemy_armor.imu_position);
        ////////////////Singer predictor//////////////////////////////
        Singer.PredictInit(dt);
        /*std::cout<<"predict_front:"<<*/Singer.predict(false)/*<<std::endl*/;
        /*std::cout<<"correct:"<<*/Singer.correct(measure)/*<<std::endl*/;
        Singer.PredictInit(all_time);
        Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
        std::cout<<"result:"<<predicted_result<<std::endl;
        predicted_position << predicted_result(0,0),predicted_result(3,0),enemy_armor.imu_position(2,0);
        bullet_point = AS.airResistanceSolve(predicted_position);
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
        std::cout<<"pixelPos"<<pixelPos<<std::endl;
        circle(AS._src,pixelPos,5,cv::Scalar(0,0,255),-1);
        cv::imshow("_src",AS._src);
//        if(tracker_state == LOSING)cv::waitKey(0);
        ////////////////Singer predictor//////////////////////////////
        return true;
    }
    else if (tracker_state == DETECTING)
    {
//        enemy_armor.imu_position = pixel2imu(enemy_armor);
        circle(AS._src,enemy_armor.center,5,cv::Scalar(255,0,0),-1);
        Eigen::Matrix<double,2,1> measure(enemy_armor.imu_position(0,0),enemy_armor.imu_position(1,0));
        std::cout<<"measurex:"<<measure[0]<<std::endl;
        std::cout<<"measurey:"<<measure[1]<<std::endl;
        double all_time = SHOOT_DELAY + AS.getFlyTime(enemy_armor.imu_position);
        ////////////////Singer predictor//////////////////////////////
        Singer.PredictInit(dt);
        /*std::cout<<"predict_front:"<<*/Singer.predict(false)/*<<std::endl*/;
        /*std::cout<<"correct:"<<*/Singer.correct(measure)/*<<std::endl*/;
        Singer.PredictInit(all_time);
        Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
        std::cout<<"result:"<<predicted_result<<std::endl;
        predicted_position << predicted_result(0,0),predicted_result(3,0),enemy_armor.imu_position(2,0);
        bullet_point = AS.airResistanceSolve(enemy_armor.imu_position);
        cv::Point2f pixelPos = AS.imu2pixel(bullet_point);
        //        std::cout<<"pixelPos"<<pixelPos<<std::endl;
        circle(AS._src,pixelPos,5,cv::Scalar(0,0,255),-1);
        cv::imshow("_src",AS._src);
//        cv::waitKey(0);
        ////////////////Singer predictor//////////////////////////////
        locate_target = false;
        return false;
    }
    else
    {
        locate_target = false;
        cv::imshow("_src",AS._src);
//        cv::waitKey(0);
        return false;
    }
}

bool ArmorTracker::locateEnemy(std::vector<Armor> &armors, double time)
{
    if(!locate_target)
    {
        if(initial(armors))
        {
            locate_target = true;
            return true;
        }
        else
        {
            locate_target = false;
            return false;
        }
    }
    else
    {
        if (t == -1)
        {
            t = time;
            return false;
        }
        double dt = (time - t) / (double)cv::getTickFrequency();
//        std::cout<<"dt:"<<dt<<std::endl;
        t = time;
        if(!selectEnemy2(armors,dt))
        {
            return false;
        }

        if(!estimateEnemy(dt))
        {
            return false;
        }

        AS.yawPitchSolve(bullet_point);

        return true;
    }
}


}