#include "dexi_camera/raw_camera_node.hpp"
#include <vector>

namespace dexi_camera
{

RawCameraNode::RawCameraNode()
: Node("raw_camera_node")
{
    // Declare parameters
    this->declare_parameter("camera_id", 0);
    this->declare_parameter("camera_width", 640);
    this->declare_parameter("camera_height", 480);
    this->declare_parameter("timer_interval", 0.033);  // ~30 Hz
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("camera_info_url", "");

    // Get parameters
    camera_id_ = this->get_parameter("camera_id").as_int();
    camera_width_ = this->get_parameter("camera_width").as_int();
    camera_height_ = this->get_parameter("camera_height").as_int();
    timer_interval_ = this->get_parameter("timer_interval").as_double();
    camera_name_ = this->get_parameter("camera_name").as_string();
    camera_info_url_ = this->get_parameter("camera_info_url").as_string();

    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw", 10);

    // Initialize camera
    cap_.open(camera_id_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);

    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_interval_),
        std::bind(&RawCameraNode::timer_callback, this));
}

void RawCameraNode::timer_callback()
{
    cv::Mat frame;
    if (cap_.read(frame)) {
        // Create and publish message
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        msg->header.stamp = this->now();
        msg->header.frame_id = camera_name_;
        msg->height = frame.rows;
        msg->width = frame.cols;
        msg->encoding = "bgr8";
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        msg->data.assign(frame.datastart, frame.dataend);

        publisher_->publish(std::move(msg));
    }
}

} // namespace dexi_camera

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_camera::RawCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 