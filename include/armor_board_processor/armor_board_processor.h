#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
using namespace cv;

class ArmorProcessor 
{
public:
    ArmorProcessor(rclcpp::Node::SharedPtr node): node_(node)
    {
        // 订阅图像话题
        image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            bind(&ArmorProcessor::tf_callback, this, placeholders::_1)
        );

        // 初始化TF广播器
        tf_broadcaster_ = make_unique<tf2_ros::TransformBroadcaster>(*node_);

        // 相机内参
        camera_matrix_ = (Mat_<double>(3,3) << 1036, 0, 640, 0, 1036, 320, 0, 0, 1);
        dist_coeffs_ = Mat::zeros(4, 1, CV_64F);

        RCLCPP_INFO(node_->get_logger(), "装甲板检测开始");
    }

private:
    // 回调函数与工具函数
    void tf_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    Mat preprocess_image(const Mat& frame);
    vector<RotatedRect> detect_armors(const Mat& frame);
    RotatedRect select_target_armor(const vector<RotatedRect>& armors);

    // 成员变量
    rclcpp::Node::SharedPtr node_;  // 共享的节点指针
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Mat camera_matrix_;  // 相机内参
    Mat dist_coeffs_;    // 畸变系数
};