#include "rclcpp/rclcpp.hpp"
#include "video_publisher/video_publisher.h"

int main(int argc, char *argv[])
{
    // 初始化ROS 2
    rclcpp::init(argc, argv);

    // 创建VideoPublisher对象并启动
    auto node = std::make_shared<VideoPublisher>();
    if (!node->start()) {
        RCLCPP_ERROR(node->get_logger(), "启动失败，退出程序");
        rclcpp::shutdown();
        return 1;
    }

    // 运行节点
    rclcpp::spin(node);

    // 关闭ROS 2
    rclcpp::shutdown();
    return 0;
}
