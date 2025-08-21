#include "armor_board_processor/armor_board_processor.h"
#include "rclcpp/logging.hpp"

using namespace std;
using namespace cv;

void ArmorProcessor::tf_callback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
    try 
    {
        // 转换ROS图像到OpenCV格式
        Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        // 预处理与装甲板检测
        Mat processed = preprocess_image(frame);
        vector<RotatedRect> armors = detect_armors(processed);

        if (!armors.empty()) 
        {
            // 选择目标装甲板并求解位姿
            RotatedRect target = select_target_armor(armors);

            // 装甲板3D坐标（相对于自身中心）
            vector<Point3f> object_points;
            object_points.emplace_back(-0.02, 0.04, 0);
            object_points.emplace_back(0.02, 0.04, 0);
            object_points.emplace_back(0.02, -0.04, 0);
            object_points.emplace_back(-0.02, -0.04, 0);

            // 图像平面角点
            vector<Point2f> image_points;
            Point2f vertices[4];
            target.points(vertices);
            for (int i = 0; i < 4; ++i) 
            {
                image_points.push_back(vertices[i]);
            }

            // PnP求解位姿
            Mat rvec, tvec;
            solvePnP(object_points, image_points, camera_matrix_, dist_coeffs_, rvec, tvec);

            // 发布TF变换
            geometry_msgs::msg::TransformStamped transform;
            transform.header = msg->header;
            transform.child_frame_id = "armor_board";

            // 平移分量
            transform.transform.translation.x = tvec.at<double>(0);
            transform.transform.translation.y = tvec.at<double>(1);
            transform.transform.translation.z = tvec.at<double>(2);

            // 旋转分量（转换为四元数）
            Mat rotate;
            Rodrigues(rvec, rotate);
            tf2::Quaternion quat;
            quat.setRPY(
                atan2(rotate.at<double>(2,1), rotate.at<double>(2,2)),
                atan2(-rotate.at<double>(2,0), sqrt(rotate.at<double>(2,1)*rotate.at<double>(2,1) + rotate.at<double>(2,2)*rotate.at<double>(2,2))),
                atan2(rotate.at<double>(1,0), rotate.at<double>(0,0))
            );
            transform.transform.rotation.x = quat.x();
            transform.transform.rotation.y = quat.y();
            transform.transform.rotation.z = quat.z();
            transform.transform.rotation.w = quat.w();

            tf_broadcaster_->sendTransform(transform);
        }
        else
        {
            RCLCPP_INFO(node_->get_logger(), "未检测到任何装甲板,不发布TF");
        }
    } 
    catch (cv_bridge::Exception& e) 
    {
        RCLCPP_ERROR(node_->get_logger(), "图形转换错误: %s", e.what());
    }
}
