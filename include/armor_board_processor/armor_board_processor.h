#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std;
using namespace cv;

class ArmorProcessor {
public:
    ArmorProcessor(rclcpp::Node::SharedPtr node);

private:
    // 回调函数与工具函数
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    Mat preprocess_image(const cv::Mat& frame);
    vector<RotatedRect> detect_armors(const Mat& frame);
    RotatedRect select_target_armor(const vector<RotatedRect>& armors);

    // 成员变量
    rclcpp::Node::SharedPtr node_;  // 共享的节点指针
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    Mat camera_matrix_;  // 相机内参
    Mat dist_coeffs_;    // 畸变系数
};