#include "dexi_camera/camera_node.hpp"
#include <vector>

namespace dexi_camera
{

MJPEGCameraNode::MJPEGCameraNode()
: Node("mjpeg_camera_node")
{
    // Declare parameters
    this->declare_parameter("camera_id", 0);
    this->declare_parameter("camera_width", 1280);
    this->declare_parameter("camera_height", 720);
    this->declare_parameter("jpeg_quality", 80);
    this->declare_parameter("timer_interval", 1.0/30.0);
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("camera_info_url", "");

    // Get parameters
    camera_id_ = this->get_parameter("camera_id").as_int();
    camera_width_ = this->get_parameter("camera_width").as_int();
    camera_height_ = this->get_parameter("camera_height").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    timer_interval_ = this->get_parameter("timer_interval").as_double();
    camera_name_ = this->get_parameter("camera_name").as_string();
    camera_info_url_ = this->get_parameter("camera_info_url").as_string();

    // Create publishers
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/" + camera_name_ + "/image_raw/compressed", 10);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/" + camera_name_ + "/camera_info", 10);

    // Initialize camera info manager
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_name_);
    if (!camera_info_url_.empty()) {
        if (!camera_info_manager_->loadCameraInfo(camera_info_url_)) {
            RCLCPP_WARN(this->get_logger(), "Failed to load camera calibration from %s", camera_info_url_.c_str());
        }
    }

    // Initialize camera
    cap_.open(camera_id_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, camera_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height_);

    // Pre-allocate frame and buffer
    frame_ = cv::Mat(camera_height_, camera_width_, CV_8UC3);
    jpeg_buffer_.reserve(camera_width_ * camera_height_ * 3);

    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_interval_),
        std::bind(&MJPEGCameraNode::timer_callback, this));
}

void MJPEGCameraNode::timer_callback()
{
    if (cap_.read(frame_)) {
        // Convert frame to JPEG format
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
        if (cv::imencode(".jpg", frame_, jpeg_buffer_, params)) {
            // Create and publish image message
            auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
            msg->header.stamp = this->now();
            msg->header.frame_id = camera_name_;
            msg->format = "jpeg";
            msg->data = jpeg_buffer_;
            publisher_->publish(std::move(msg));

            // Publish camera info
            auto camera_info = camera_info_manager_->getCameraInfo();
            camera_info.header.stamp = msg->header.stamp;
            camera_info.header.frame_id = camera_name_;
            camera_info_publisher_->publish(camera_info);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode image");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to read frame from camera");
    }
}

} // namespace dexi_camera

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_camera::MJPEGCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}