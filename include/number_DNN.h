#include <opencv2/opencv.hpp>
#include <iostream>

#define NET_PATH "/home/lmx/real/other/2022_10_21_hj_num_4.onnx"
#define TO_GRAY 1
#define INPUT_WIDTH 20
#define INPUT_HEIGHT 30
#define THRESH_CONFIDENCE 0.8

//namespace robot_detection {

    class DNN_detect{
    public:
        static cv::dnn::Net read_net(const std::string& net_path);
        static cv::Mat img_processing(cv::Mat ori_img, bool is_gray);
        static void net_forward(const cv::Mat& blob, cv::dnn::Net net, int& id, double & confidence);
    };

//}