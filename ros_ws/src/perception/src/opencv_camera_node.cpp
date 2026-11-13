#include "perception/opencv_camera_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

OpencvCameraNode::OpencvCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("opencv_camera_node", options)
{
  this->declare_parameter<std::string>("video_device", "/dev/video0");
  this->declare_parameter<std::string>("frame_id", "camera_frame");
  this->declare_parameter<int>("width", 640);
  this->declare_parameter<int>("height", 480);

  video_device_ = this->get_parameter("video_device").as_string();
  frame_id_ = this->get_parameter("frame_id").as_string();
  width_ = this->get_parameter("width").as_int();
  height_ = this->get_parameter("height").as_int();

  // Try opening video device as integer index if device string is numeric (e.g. "0")
  if (!video_device_.empty() && std::isdigit(video_device_[0])) {
    int dev_idx = std::stoi(video_device_);
    cap_.open(dev_idx, cv::CAP_V4L2);
  } else {
    cap_.open(video_device_, cv::CAP_V4L2);
  }

  if (!cap_.isOpened()) {
    // Fallback attempt without V4L2 backend flag
    if (!video_device_.empty() && std::isdigit(video_device_[0])) {
      cap_.open(std::stoi(video_device_));
    } else {
      cap_.open(video_device_);
    }
  }

  if (!cap_.isOpened()) {
    RCLCPP_ERROR(this->get_logger(), "Gagal membuka kamera pada device: %s", video_device_.c_str());
  } else {
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    RCLCPP_INFO(this->get_logger(), "Berhasil membuka kamera %s (%dx%d).", video_device_.c_str(), width_, height_);
  }

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);
  timer_ = this->create_wall_timer(33ms, std::bind(&OpencvCameraNode::captureLoop, this));
}

OpencvCameraNode::~OpencvCameraNode()
{
  if (cap_.isOpened()) {
    cap_.release();
  }
}

void OpencvCameraNode::captureLoop()
{
  if (!cap_.isOpened()) {
    return;
  }

  cv::Mat frame;
  if (!cap_.read(frame) || frame.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Gagal membaca frame dari kamera %s", video_device_.c_str());
    return;
  }

  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id_;

  cv_bridge::CvImage cv_img(header, "bgr8", frame);
  image_pub_->publish(*cv_img.toImageMsg());
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::OpencvCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
