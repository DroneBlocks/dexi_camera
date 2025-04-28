#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>

namespace dexi_camera
{

class MJPEGCameraNode : public rclcpp::Node
{
public:
    explicit MJPEGCameraNode();

private:
    void timer_callback();
    
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::VideoCapture cap_;
    
    static constexpr double TIMER_INTERVAL = 1.0/30.0;  // 30 FPS
    static constexpr int CAMERA_WIDTH = 1280;
    static constexpr int CAMERA_HEIGHT = 720;
    static constexpr int JPEG_QUALITY = 80;
};

} // namespace dexi_camera_cpp