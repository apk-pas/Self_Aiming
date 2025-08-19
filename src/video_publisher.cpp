#include "video_publisher/video_publisher.h"
#include "rclcpp/logging.hpp"

using namespace std;
using namespace cv; 

VideoPublisher::VideoPublisher() : Node("video_publisher"), current_frames_(0), total_frames_(0)
{
    // 初始化参数
    initialize_parameters();
    
    // 创建图像发布器
    image_pub_ = image_transport::create_publisher(this, "image_raw");
    
    // 打开视频源
    if (!open_video_source()) {
        RCLCPP_ERROR(this->get_logger(), "无法打开视频源，节点将退出");
        rclcpp::shutdown();
        return;
    }
    
    // 创建定时器，控制发布帧率
    auto period = chrono::milliseconds(1000 / frame_rate_);
    timer_ = this->create_wall_timer(
        period,
        bind(&VideoPublisher::publish_frame, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "视频发布节点已启动，输入类型: %s", input_type_.c_str());
    if (input_type_ == "video") {
        RCLCPP_INFO(this->get_logger(), "视频总帧数: %d,将循环播放", total_frames_);
    }
}

VideoPublisher::~VideoPublisher()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void VideoPublisher::initialize_parameters()
{
    // 声明参数并设置默认值
    this->declare_parameter<string>("input_type", "camera");
    this->declare_parameter<string>("video_path", "");
    this->declare_parameter<int>("camera_id", 0);
    this->declare_parameter<int>("frame_rate", 30);
    
    // 获取参数值
    this->get_parameter("input_type", input_type_);
    this->get_parameter("video_path", video_path_);
    this->get_parameter("camera_id", camera_id_);
    this->get_parameter("frame_rate", frame_rate_);
}

bool VideoPublisher::open_video_source()
{
    if (input_type_ == "camera") {
        // 打开USB相机
        if (!cap_.open(camera_id_)) {
            RCLCPP_ERROR(this->get_logger(), "无法打开USB相机,ID: %d", camera_id_);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "成功打开USB相机,ID: %d", camera_id_);
        return true;
    } 
    else if (input_type_ == "video") {
        // 打开视频文件
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "视频路径不能为空");
            return false;
        }
        
        if (!cap_.open(video_path_)) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str());
            return false;
        }
        
        // 获取视频总帧数
        total_frames_ = static_cast<int>(cap_.get(CAP_PROP_FRAME_COUNT));
        RCLCPP_INFO(this->get_logger(), "成功打开视频文件: %s", video_path_.c_str());
        return true;
    }
    
    RCLCPP_ERROR(this->get_logger(), "无效的输入类型: %s,支持的类型为 'camera' 或 'video'", input_type_.c_str());
    return false;
}

void VideoPublisher::publish_frame()
{
    Mat frame;
    cap_ >> frame;
    
    // 处理视频文件的循环播放
    if (input_type_ == "video") {
        current_frames_++;
        
        // 检查是否播放完毕
        if (frame.empty() || current_frames_ >= total_frames_) {
            RCLCPP_INFO(this->get_logger(), "视频播放完毕,将重新开始 (总帧数: %d)", total_frames_);
            reset_video();
            cap_ >> frame;
            current_frames_ = 1;
        }
    }
    // 处理相机无数据的情况
    else if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "无法获取相机帧");
        return;
    }
    
    // 转换为ROS图像消息并发布
    try {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        image_pub_.publish(msg);
        
        // 视频模式下打印当前播放进度
        if (input_type_ == "video" && current_frames_ % 30 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), 
                               *this->get_clock(), 1000,  // 每1秒最多打印一次
                               "播放进度: %d/%d 帧", current_frames_, total_frames_);
        }
    } 
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge转换错误: %s", e.what());
    }
}

void VideoPublisher::reset_video()
{
    if (input_type_ == "video" && cap_.isOpened()) {
        cap_.set(CAP_PROP_POS_FRAMES, 0);  // 重置到第一帧
        current_frames_ = 0;
    }
}
