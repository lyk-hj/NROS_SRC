#include <opencv2/opencv.hpp>
#include <iostream>

#define NET_PATH "../model/2023_1_5_hj_num_8.onnx"
#define INPUT_WIDTH 20
#define INPUT_HEIGHT 30
#define TO_GRAY 1
#define THRESH_CONFIDENCE 0.843

namespace robot_detection {

class DNN_detect{
public:
    static cv::dnn::Net read_net(const std::string& net_path);
    static cv::Mat img_processing(cv::Mat ori_img, bool is_gray);
    static void net_forward(const cv::Mat& blob, cv::dnn::Net net, int& id, double & confidence);
};

}