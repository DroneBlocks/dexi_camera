#ifndef DEXI_CAMERA_CAMERA_NODE_HPP_
#define DEXI_CAMERA_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <camera_info_manager/camera_info_manager.hpp>

namespace dexi_camera
{

class MJPEGCameraNode : public rclcpp::Node
{
public:
    MJPEGCameraNode();

private:
    void timer_callback();
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
};

} // namespace dexi_camera

#endif // DEXI_CAMERA_CAMERA_NODE_HPP_