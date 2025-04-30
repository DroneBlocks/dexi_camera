#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <camera_info_manager/camera_info_manager.hpp>

namespace dexi_camera
{

class RawCameraNode : public rclcpp::Node
{
public:
    explicit RawCameraNode();

private:
    void timer_callback();
    void load_camera_calibration();
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    cv::Mat frame_;
    
    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    
    // Parameters
    int camera_id_;
    int camera_width_;
    int camera_height_;
    double timer_interval_;
    std::string camera_name_;
    std::string camera_info_url_;
};

} 