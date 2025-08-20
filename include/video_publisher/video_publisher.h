#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

class VideoPublisher : public rclcpp::Node
{
public:
    VideoPublisher();

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
