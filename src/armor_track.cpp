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

    change_aim_cnt = 0;
    change_aim_threshold = 20;
    isChangeSameID = false;
}

// 初始化，选择最优装甲板，设置卡尔曼的F和x_1
void ArmorTracker::initial(std::vector<Armor> &find_armors)
{
    if(find_armors.empty())
    {
        return ;
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
}

// dt是两帧之间时间间隔, 跟得住目标
Armor ArmorTracker::selectEnemy2(std::vector<Armor> &find_armors, double dt)
{

    Eigen::VectorXd kf_pre = KF.predict(dt,false);

    Armor matched_armor;
    bool matched = false;
    predicted_enemy = kf_pre;

    if(!find_armors.empty())
    {
        double min_position_diff = DBL_MAX;
        for(auto & armor : find_armors)
        {
            Eigen::Vector3d position_vec = pixel2imu(armor);
            Eigen::Vector3d predicted_position = {position_vec(0,0),
                                                  position_vec(2,0),
                                                  position_vec(4,0)};
            double position_diff = (predicted_position - position_vec).norm();
            if (position_diff < min_position_diff) {
                min_position_diff = position_diff;
                matched_armor = armor;
            }
        }

        if (min_position_diff < new_old_threshold) {
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
                    armor.imu_position = pixel2imu(armor);
                    Eigen::VectorXd pos_speed(6);
                    pos_speed << armor.imu_position, predicted_speed;
                    KF.setPosAndSpeed(armor.imu_position, predicted_speed);
                    predicted_enemy = pos_speed;
                    break;
                }
            }
        }

    }

    predicted_speed = {kf_pre(0,0), kf_pre(2,0), kf_pre(4,0)};

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
    KF.setPosAndSpeed(predicted_enemy);

    return matched_armor;
}

// 对处于跟踪和正在丢失状态时做 预测，引入各种时间
Eigen::Vector3d ArmorTracker::estimateEnemy(Armor &armor, double dt)
{
    //
    if(tracker_state == TRACKING || tracker_state == LOSING)
    {
        // enemy_armor get real information to predicted by singer
        Eigen::Vector3d imuPos = pixel2imu(armor);
        Eigen::Matrix<double,2,1> measure(imuPos(0,0),imuPos(1,0));
        double all_time = SHOOT_DELAY + getFlyTime(armor.camera_position);
        ////////////////Singer predictor//////////////////////////////
        Singer.PredictInit(dt);
        Singer.predict(false);
        Singer.correct(measure);
        Singer.PredictInit(all_time);
        Eigen::Matrix<double,6,1> predicted_result = Singer.predict(true);
        predicted_position << predicted_result(0,0),predicted_result(3,0),imuPos(2,0);
        ////////////////Singer predictor//////////////////////////////

        Eigen::Vector3d pre_pos_cam = imu2cam(predicted_position);
        Eigen::Vector3d bullet_point = airResistanceSolve(pre_pos_cam);
        return bullet_point;
    }
    else
    {
        locate_target = false;
        return {};
    }

}

bool ArmorTracker::locateEnemy(std::vector<Armor> &armors, double time)
{
    if(!locate_target)
    {
        initial(armors);
        locate_target = true;
    }

    if (t == -1)
    {
        t = time;
        return false;
    }

    double dt = (time - t) / (double)cv::getTickFrequency();
    t = time;

    Armor armor = selectEnemy2(armors,dt);

    Eigen::Vector3d bullet_point = estimateEnemy(armor, dt);

    yawPitchSolve(bullet_point);

    return true;
}


}