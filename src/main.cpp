#include "rclcpp/rclcpp.hpp"
#include "video_publisher/video_publisher.h"

using namespace std;

int main(int argc, char *argv[])
{
    // 初始化ROS
    rclcpp::init(argc, argv);
    
    // 创建并运行视频发布节点
    auto node = make_shared<VideoPublisher>();
    rclcpp::spin(node);
    
    // 关闭ROS
    rclcpp::shutdown();
    return 0;
}
