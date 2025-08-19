#ifndef VIDEO_PUBLISHER_VIDEO_PUBLISHER_H
#define VIDEO_PUBLISHER_VIDEO_PUBLISHER_H

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class VideoPublisher : public rclcpp::Node
{
public:
    // 构造函数
    VideoPublisher();
    // 析构函数
    ~VideoPublisher() override;

    // 启动发布流程
    bool start();

private:
    // 声明ROS参数
    void declare_parameters();
    // 读取参数
    void read_parameters();
    // 打开视频源（相机或视频文件）
    bool open_video_source();
    // 发布图像帧的回调函数
    void publish_frame();

    // OpenCV视频捕获对象
    cv::VideoCapture cap_;
    // 图像发布器
    image_transport::Publisher image_pub_;
    // 定时器（控制发布频率）
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数存储
    std::string input_type_;    // "camera" 或 "video"
    std::string video_path_;    // 视频文件路径
    int camera_id_;             // 相机ID
    int frame_rate_;            // 发布帧率
    bool is_running_;           // 运行状态标记
};

#endif  // VIDEO_PUBLISHER_VIDEO_PUBLISHER_H
