#include "armor_track.h"
#include "gimbal_control.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define DRAW_CENTER_CIRCLE

namespace robot_detection {

ArmorTracker::ArmorTracker()
{
    locate_target = false;
    enemy_armor = Armor();

    tracker_state = MISSING;
    tracking_id = 0;

    find_aim_cnt = 0;
    find_threshold = 30;

    lost_aim_cnt = 0;
    lost_threshold = 50;

    new_old_threshold = 10; //cm

    isChangeSameID = false;
}

void ArmorTracker::reset()
{
    //  Singer.reset();
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

    Armor best_armor;
    int maxGrade = 0;
    for(auto & armor : find_armors)
    {
        if (armor.grade > maxGrade) {
            maxGrade = armor.grade;
            best_armor = armor;
        }
    }

    // select enemy
    enemy_armor = best_armor;
    tracker_state = DETECTING;
    tracking_id = enemy_armor.id;

    enemy_armor.imu_position = pixel2imu(enemy_armor);
    KF.setXPost(enemy_armor.imu_position);
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
            Eigen::Vector3d cur = pixel2imu(armor);
            Eigen::Vector3d pre = predicted_enemy.head(3);
            double position_diff = (pre - cur).norm();
            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                matched_armor = armor;
            }
        }

        if (min_position_diff < new_old_threshold && matched_armor.id == tracking_id) {
            matched = true;
            Eigen::Vector3d position_vec = pixel2imu(matched_armor);
            predicted_enemy = KF.correct(position_vec);
        }
        else
        {
            // 本帧内是否有相同ID
            for (auto & armor : find_armors) {
                if (armor.id == tracking_id) {
                    matched = true;
                    KF.reset();
                    armor.imu_position = pixel2imu(armor);
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
    //
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        // enemy_armor get real information to predicted by singer
//        Eigen::Vector3d imuPos = pixel2imu(armor);
        Eigen::Matrix<double,2,1> measure(enemy_armor.imu_position(0,0),enemy_armor.imu_position(1,0));
        double all_time = SHOOT_DELAY + fly_time;
        ////////////////Singer predictor//////////////////////////////
        Singer.PredictInit(dt);
        Singer.predict(false);
        Singer.correct(measure);
        Singer.PredictInit(all_time);
        Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
        predicted_position << predicted_result(0,0),predicted_result(3,0),enemy_armor.imu_position(2,0);
        ////////////////Singer predictor//////////////////////////////

        Eigen::Vector3d pre_pos_cam = imu2cam(predicted_position);
        bullet_point = airResistanceSolve(pre_pos_cam);
        return true;
    }
    else
    {
        locate_target = false;
        return false;
    }

}

bool ArmorTracker::locateEnemy(std::vector<Armor> &armors, double time)
{
//    if(!locate_target)
//    {
//        initial(armors);
//        locate_target = true;
//    }
//
//    if (t == -1)
//    {
//        t = time;
//        return false;
//    }
//
//    double dt = (time - t) / (double)cv::getTickFrequency();
//    t = time;
//
//    Armor armor = selectEnemy2(armors,dt);
//
//    Eigen::Vector3d bullet_point = estimateEnemy(armor, dt);
//
//    yawPitchSolve(bullet_point);
//
//    return true;
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
        t = time;
        if(!selectEnemy2(armors,dt))
        {
            return false;
        }

        if(!estimateEnemy(dt))
        {
            return false;
        }

        yawPitchSolve(bullet_point);
        return true;
    }
}


}