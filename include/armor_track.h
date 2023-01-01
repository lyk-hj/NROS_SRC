#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "armor_detection.hpp"
#include "Singer.hpp"
#include "gimbal_control.h"
#include "kal_filter.hpp"

// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

namespace robot_detection {

enum TrackerState {
    MISSING,     // 没有目标，跳过该部分
    DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好卡尔曼
    LOSING,      // 处于丢失状态，还会保留预测
    TRACKING,    // 处于跟踪状态
};



class ArmorTracker : public AngleSolve
{
public:
    cv::Mat _src;

    ArmorTracker();

    void initial(std::vector<Armor> &find_armors);
    Armor selectEnemy2(std::vector<Armor> &find_armors, double dt);

    //
    Eigen::Vector3d estimateEnemy(Armor &armor, double dt);

    bool locateEnemy(std::vector<Armor> &armors, double time);
private:
    Armor enemy_armor;//最终选择的装甲板

    Skalman Singer;

    kal_filter KF;

    bool locate_target;
    bool isChangeSameID;

    int tracker_state;  // 此时跟踪器的状态
    int tracking_id;  // 跟踪的敌方ID

    int find_aim_cnt;
    int find_threshold;

    int lost_aim_cnt;  // 丢失目标计数
    int lost_threshold;

    int change_aim_cnt;

    double change_aim_threshold;

    double new_old_threshold; // 新旧坐标的距离阈值
    double cur_pre_threshold; // 当前和预测的坐标点的距离阈值

    double t;

    Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度
    Eigen::Vector3d predicted_speed;  // 预测得到的速度???
    Eigen::Matrix<double,6,1> predicted_enemy;
};

}