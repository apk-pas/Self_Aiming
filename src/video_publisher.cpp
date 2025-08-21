#include "video_publisher/video_publisher.h"
#include "rclcpp/logging.hpp"

using namespace std;
using namespace cv; 

bool VideoPublisher::open_video_source()
{
    if (input_type_ == "camera") 
    {
        // 打开USB相机
        if (!cap_.open(camera_id_)) 
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开USB相机%d", camera_id_);
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "成功打开USB相机%d", camera_id_);
        return true;
    } 
    else if (input_type_ == "video") 
    {
        // 打开视频文件
        if (video_path_.empty()) 
        {
            RCLCPP_ERROR(this->get_logger(), "视频路径不能为空");
            return false;
        }
        if (!cap_.open(video_path_)) 
        {
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
    if (input_type_ == "video") 
    {
        current_frames_++;
        
        // 检查是否播放完毕
        if (frame.empty() || current_frames_ >= total_frames_) 
        {
            reset_video();
            cap_ >> frame;
            current_frames_ = 1;
        }
    }
    // 处理相机无数据的情况
    else if (frame.empty()) 
    {
        RCLCPP_WARN(this->get_logger(), "无法获取相机帧");
        return;
    }
    
    // 转换为ROS图像消息并发布
    try 
    {
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera_frame";
        image_pub_.publish(msg);
    } 
    catch (cv_bridge::Exception& e) 
    {
        RCLCPP_ERROR(this->get_logger(), "视频格式转换错误: %s", e.what());
    }
}

void VideoPublisher::reset_video()
{
    if (input_type_ == "video" && cap_.isOpened()) 
    {
        cap_.set(CAP_PROP_POS_FRAMES, 0);  // 重置到第一帧
        current_frames_ = 0;
    }
}
