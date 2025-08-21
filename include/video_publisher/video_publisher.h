#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher(): rclcpp::Node("video_publisher")
    {
    // 声明参数并设置默认值
    this->declare_parameter<string>("input_type", "video");
    this->declare_parameter<string>("video_path", "/home/apk/rosdemo/src/self_aiming/videos/test.mp4");
    this->declare_parameter<int>("camera_id", 0);
    this->declare_parameter<int>("frame_rate", 30);
    
    // 获取参数值
    this->get_parameter("input_type", input_type_);
    this->get_parameter("video_path", video_path_);
    this->get_parameter("camera_id", camera_id_);
    this->get_parameter("frame_rate", frame_rate_);

    // 创建图像发布器
    image_pub_ = image_transport::create_publisher(this, "image_raw");
    
    // 打开视频源
    if (!open_video_source()) 
    {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频,退出程序");
        rclcpp::shutdown();
        return;
    }
    
    // 创建定时器，控制发布帧率
    auto period = chrono::milliseconds(1000 / frame_rate_);
    timer_ = this->create_wall_timer(
        period,
        bind(&VideoPublisher::publish_frame, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "%s发布节点已启动", input_type_.c_str());
    }

private:
    // 打开视频源
    bool open_video_source();
    
    // 发布图像帧
    void publish_frame();
    
    // 循环播放
    void reset_video();
    
    // 视频捕获对象
    cv::VideoCapture cap_;
    
    // 图像发布器
    image_transport::Publisher image_pub_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 参数
    string input_type_;
    string video_path_;
    int camera_id_;
    int frame_rate_;
    int current_frames_ = 0;  // 视频总帧数
    int total_frames_ = 0; // 当前播放帧数
};
