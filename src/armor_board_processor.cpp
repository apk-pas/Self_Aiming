#include "armor_board_processor/armor_board_processor.h"
#include "video_publisher/video_publisher.h"
#include "rclcpp/logging.hpp"

using namespace std;
using namespace cv;

ArmorProcessor::ArmorProcessor(rclcpp::Node::SharedPtr node) : node_(node){
    // 订阅图像话题
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10,
        bind(&ArmorProcessor::image_callback, this, placeholders::_1)
    );

    // 初始化TF广播器
    tf_broadcaster_ = make_unique<tf2_ros::TransformBroadcaster>(*node_);

    // 相机内参（建议从参数服务器加载）
    camera_matrix_ = (Mat_<double>(3,3) << 800, 0, 320, 0, 800, 240, 0, 0, 1);
    dist_coeffs_ = Mat::zeros(5, 1, CV_64F);

    RCLCPP_INFO(node_->get_logger(), "装甲板检测开始");
}

void ArmorProcessor::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        // 转换ROS图像到OpenCV格式
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 预处理与装甲板检测
        Mat processed = preprocess_image(frame);
        vector<RotatedRect> armors = detect_armors(processed);

        if (!armors.empty()) {
            // 选择目标装甲板并求解位姿
            RotatedRect target = select_target_armor(armors);

            // 装甲板3D坐标（相对于自身中心）
            std::vector<cv::Point3f> object_points;
            object_points.emplace_back(-0.1, 0.05, 0);
            object_points.emplace_back(0.1, 0.05, 0);
            object_points.emplace_back(0.1, -0.05, 0);
            object_points.emplace_back(-0.1, -0.05, 0);

            // 图像平面角点
            vector<Point2f> image_points;
            Point2f vertices[4];
            target.points(vertices);
            for (int i = 0; i < 4; ++i) {
                image_points.push_back(vertices[i]);
            }

            // PnP求解位姿
            Mat rvec, tvec;
            solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

            // 发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header = msg->header;
            transform.child_frame_id = "armor";

            // 平移分量
            transform.transform.translation.x = tvec.at<double>(0);
            transform.transform.translation.y = tvec.at<double>(1);
            transform.transform.translation.z = tvec.at<double>(2);

            // 旋转分量（转换为四元数）
            Mat rot_mat;
            Rodrigues(rvec, rot_mat);
            tf2::Quaternion quat;
            quat.setRPY(
                atan2(rot_mat.at<double>(2,1), rot_mat.at<double>(2,2)),
                atan2(-rot_mat.at<double>(2,0), sqrt(rot_mat.at<double>(2,1)*rot_mat.at<double>(2,1) + rot_mat.at<double>(2,2)*rot_mat.at<double>(2,2))),
                atan2(rot_mat.at<double>(1,0), rot_mat.at<double>(0,0))
            );
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            transform.transform.rotation.w = quat.w();

            tf_broadcaster_->sendTransform(transform);
        }
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "图形转换错误: %s", e.what());
    }
}

Mat ArmorProcessor::preprocess_image(const Mat& frame) {
    Mat hsv, red_mask1, red_mask2, red_mask;
    cvtColor(frame, hsv, COLOR_BGR2HSV);
    inRange(hsv, Scalar(0, 120, 70), Scalar(10, 255, 255), red_mask1);
    inRange(hsv, Scalar(170, 120, 70), Scalar(180, 255, 255), red_mask2);
    red_mask = red_mask1 | red_mask2;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(red_mask, red_mask, MORPH_CLOSE, kernel);
    return red_mask;
}

vector<RotatedRect> ArmorProcessor::detect_armors(const Mat& frame) {
    vector<RotatedRect> armors;
    vector<vector<Point>> contours;
    findContours(frame, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& contour : contours) {
        if (contourArea(contour) < 50) continue;
        RotatedRect rect = minAreaRect(contour);
        float ratio = max(rect.size.width, rect.size.height) / min(rect.size.width, rect.size.height);
        if (ratio > 3 && ratio < 8) {
            armors.push_back(rect);
        }
    }
    return armors;
}

RotatedRect ArmorProcessor::select_target_armor(const vector<RotatedRect>& armors) {
    int max_idx = 0;
    float max_area = armors[0].size.area();
    for (size_t i = 1; i < armors.size(); ++i) {
        float area = armors[i].size.area();
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    return armors[max_idx];
}
