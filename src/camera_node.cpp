#include "dexi_camera/camera_node.hpp"
#include <vector>
#include <camera_info_manager/camera_info_manager.hpp>

namespace dexi_camera
{

MJPEGCameraNode::MJPEGCameraNode()
: Node("camera_node")
{
    // Declare parameters
    this->declare_parameter("camera_id", 0);
    this->declare_parameter("camera_width", 1280);
    this->declare_parameter("camera_height", 720);
    this->declare_parameter("jpeg_quality", static_cast<int64_t>(75));
    this->declare_parameter("timer_interval", 1.0/30.0);
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("camera_info_url", "");

    // Get parameters
    int camera_id = this->get_parameter("camera_id").as_int();
    int width = this->get_parameter("camera_width").as_int();
    int height = this->get_parameter("camera_height").as_int();
    int64_t jpeg_quality = this->get_parameter("jpeg_quality").as_int();
    double timer_interval = this->get_parameter("timer_interval").as_double();
    std::string camera_name = this->get_parameter("camera_name").as_string();
    std::string camera_info_url = this->get_parameter("camera_info_url").as_string();

    // Create publisher with the configured camera name
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        camera_name + "/image_raw/compressed", 10);

    // Initialize camera info manager
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name);
    if (!camera_info_url.empty()) {
        if (camera_info_manager_->loadCameraInfo(camera_info_url)) {
            RCLCPP_INFO(this->get_logger(), "Loaded camera calibration from %s", camera_info_url.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to load camera calibration from %s", camera_info_url.c_str());
        }
    }

    // Create camera info publisher
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
        camera_name + "/camera_info", 10);

    // Initialize camera
    cap_.open(camera_id);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera with ID: %d", camera_id);
        return;
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // Create timer with configured interval
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(timer_interval),
        std::bind(&MJPEGCameraNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Camera node initialized with parameters:");
    RCLCPP_INFO(this->get_logger(), "  Camera ID: %d", camera_id);
    RCLCPP_INFO(this->get_logger(), "  Resolution: %dx%d", width, height);
    RCLCPP_INFO(this->get_logger(), "  JPEG Quality: %ld", jpeg_quality);
    RCLCPP_INFO(this->get_logger(), "  Timer Interval: %f", timer_interval);
    RCLCPP_INFO(this->get_logger(), "  Camera Name: %s", camera_name.c_str());
    RCLCPP_INFO(this->get_logger(), "  Camera Info URL: %s", camera_info_url.c_str());
}

void MJPEGCameraNode::timer_callback()
{
    cv::Mat frame;
    if (cap_.read(frame)) {
        // Convert frame to JPEG format
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, static_cast<int>(this->get_parameter("jpeg_quality").as_int())};
        cv::imencode(".jpg", frame, buffer, params);

        // Create and publish image message
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->format = "jpeg";
        msg->data = buffer;
        publisher_->publish(std::move(msg));

        // Publish camera info
        auto camera_info = camera_info_manager_->getCameraInfo();
        camera_info.header.stamp = this->now();
        camera_info_pub_->publish(camera_info);
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