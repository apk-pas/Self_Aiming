#include "video_publisher/video_publisher.h"
#include "rclcpp/logging.hpp"

VideoPublisher::VideoPublisher() : Node("video_publisher"), is_running_(false)
{
    // 声明并读取参数
    declare_parameters();
    read_parameters();

    // 创建图像发布器（话题名：/image_raw，与RViz默认匹配）
    image_pub_ = image_transport::create_publisher(this, "image_raw");

    RCLCPP_INFO(this->get_logger(), "VideoPublisher节点初始化完成");
}

VideoPublisher::~VideoPublisher()
{
    if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(this->get_logger(), "视频源已关闭");
    }
}

void VideoPublisher::declare_parameters()
{
    // 声明参数（带默认值）
    this->declare_parameter<std::string>("input_type", "video");
    this->declare_parameter<std::string>("video_path", "/home/apk/rosdemo/src/self_aiming/videos/test.mp4");
    this->declare_parameter<int>("camera_id", 0);
    this->declare_parameter<int>("frame_rate", 30);
}

void VideoPublisher::read_parameters()
{
    // 读取参数值
    input_type_ = this->get_parameter("input_type").as_string();
    video_path_ = this->get_parameter("video_path").as_string();
    camera_id_ = this->get_parameter("camera_id").as_int();
    frame_rate_ = this->get_parameter("frame_rate").as_int();

    // 打印参数信息
    RCLCPP_INFO(this->get_logger(), "输入类型: %s", input_type_.c_str());
    if (input_type_ == "video") {
        RCLCPP_INFO(this->get_logger(), "视频路径: %s", video_path_.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "相机ID: %d", camera_id_);
    }
    RCLCPP_INFO(this->get_logger(), "发布帧率: %d FPS", frame_rate_);
}

bool VideoPublisher::open_video_source()
{
    if (input_type_ == "camera") {
        // 打开USB相机
        if (!cap_.open(camera_id_)) {
            RCLCPP_ERROR(this->get_logger(), "无法打开相机 (ID: %d)", camera_id_);
            return false;
        }
    } else if (input_type_ == "video") {
        // 打开视频文件
        if (video_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "视频路径为空,请指定video_path参数");
            return false;
        }
        if (!cap_.open(video_path_)) {
            RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s", video_path_.c_str());
            return false;
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "无效的输入类型: %s,支持'camera'或'video'", input_type_.c_str());
        return false;
    }
    return true;
}

void VideoPublisher::publish_frame()
{
    cv::Mat frame;
    cap_ >> frame;  // 读取一帧图像

    if (frame.empty()) {
        RCLCPP_WARN(this->get_logger(), "无法获取图像帧（可能已到达视频末尾）");
        return;
    }

    // 将OpenCV图像转换为ROS图像消息
    sensor_msgs::msg::Image::SharedPtr msg = 
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
    msg->header.stamp = this->now();  // 设置时间戳
    msg->header.frame_id = "camera_frame";  // 设置坐标系（RViz中需对应）

    // 发布图像
    image_pub_.publish(msg);
}

bool VideoPublisher::start()
{
    // 打开视频源
    if (!open_video_source()) {
        return false;
    }

    // 创建定时器，按指定帧率调用publish_frame
    auto period = std::chrono::milliseconds(1000 / frame_rate_);
    timer_ = this->create_wall_timer(
        period,
        std::bind(&VideoPublisher::publish_frame, this)
    );

    is_running_ = true;
    RCLCPP_INFO(this->get_logger(), "视频发布已启动，话题: /image_raw");
    return true;
}
