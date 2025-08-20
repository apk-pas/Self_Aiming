#include "rclcpp/rclcpp.hpp"
#include "video_publisher/video_publisher.h"
#include "armor_board_processor/armor_board_processor.h"

using namespace std;

int main(int argc, char *argv[])
{
    // 初始化ROS
    rclcpp::init(argc, argv);
    
    // 创建并运行视频发布节点
    auto node = make_shared<VideoPublisher>();

    //传入视频节点的指针，共享同一个节点
    ArmorProcessor armor_processor(node);

    //运行节点
    rclcpp::spin(node);
    
    // 关闭ROS
    rclcpp::shutdown();
    return 0;
}
