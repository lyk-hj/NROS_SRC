#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include "robot_status.h"
#include "armor_detection.hpp"
#include "armor_prediction.h"
#include "Singer.hpp"
#include "gimbal_control.h"

// 目的：通过读取进来的armors，筛选出同ID和上一帧的的装甲板，做跟踪

//namespace robot_detection {

    enum TrackerState {
        MISSING,     // 没有目标，跳过该部分
        DETECTING,   // 还未开始跟踪，作为跟踪第一帧的切换，初始化好卡尔曼
        LOSING,      // 处于丢失状态，还会保留预测
        TRACKING,    // 处于跟踪状态
    };

    class ArmorTracker
    {
    public:
        cv::Mat _src;
        float pitch;
        float yaw;
        double t;

        ArmorTracker();

        void reset();

        bool initial(std::vector<Armor> find_armors);

        bool selectEnemy2(std::vector<Armor> find_armors, double dt);

        //
        bool estimateEnemy(double dt);

        bool locateEnemy(cv::Mat src, std::vector<Armor> armors, double time);

//    private:

        Armor enemy_armor;
        Eigen::Vector3d bullet_point;

        AngleSolve AS;
        // Q和R分别代表对预测值和测量值的置信度（反比），通过影响卡尔曼增益K的值，影响预测值和测量值的权重。越大的R代表越不相信测量值。
        KalmanFilter KF;
        Skalman Singer;

        bool locate_target;

        int tracker_state;  // 此时跟踪器的状态
        int tracking_id;  // 跟踪的敌方ID

        int find_aim_cnt;
        int find_threshold;

        int lost_aim_cnt;  // 丢失目标计数
        int lost_threshold;

        bool isChangeSameID;  // 单个目标不用切换

        double new_old_threshold; // 新旧坐标的距离阈值
        double cur_pre_threshold; // 当前和预测的坐标点的距离阈值
        Eigen::Matrix<double,6,1> predicted_enemy;
        Eigen::Vector3d predicted_position;  // 预测的坐标，也是要发送给电控角度的坐标计算的角度
        Eigen::Vector3d predicted_speed;  // 预测得到的速度

        double countArmorIoU(Armor armor1, Armor armor2);
    };

//}
//        // 本帧的真实坐标和预测的下一帧坐标
//        double cur_pre_distance = (predicted_position - find_armors[0].current_position).norm();
//        if(cur_pre_distance > cur_pre_threshold)
//        {
//            lost_aim_cnt++;
//            if(lost_aim_cnt > lost_threshold)
//            {
//                lost_aim_cnt = 0;
//                tracking_id = 0;
//                tracker_state = DETECTING;
//                enemy_armors.clear();
//            }
//            return ;
//        }