#include "dexi_camera/camera_node.hpp"
#include <vector>

namespace dexi_camera
{

MJPEGCameraNode::MJPEGCameraNode()
: Node("mjpeg_camera_node")
{
    // Create publisher
    publisher_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/cam0/image_raw/compressed", 10);

    // Initialize camera
    cap_.open(0);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    // Set camera properties
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);

    // Create timer
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(TIMER_INTERVAL),
        std::bind(&MJPEGCameraNode::timer_callback, this));
}

void MJPEGCameraNode::timer_callback()
{
    cv::Mat frame;
    if (cap_.read(frame)) {
        // Convert frame to JPEG format
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, JPEG_QUALITY};
        cv::imencode(".jpg", frame, buffer, params);

        // Create and publish message
        auto msg = std::make_unique<sensor_msgs::msg::CompressedImage>();
        msg->header.stamp = this->now();
        msg->format = "jpeg";
        msg->data = buffer;

        publisher_->publish(std::move(msg));
    }
}

} // namespace dexi_camera

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<dexi_camera_cpp::MJPEGCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}