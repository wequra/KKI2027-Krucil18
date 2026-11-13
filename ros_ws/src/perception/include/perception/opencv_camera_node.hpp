#ifndef PERCEPTION__OPENCV_CAMERA_NODE_HPP_
#define PERCEPTION__OPENCV_CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <string>

namespace dagozilla
{

class OpencvCameraNode : public rclcpp::Node
{
public:
  explicit OpencvCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OpencvCameraNode() override;

private:
  void captureLoop();

  std::string video_device_{"/dev/video0"};
  std::string frame_id_{"camera_frame"};
  int width_{640};
  int height_{480};

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dagozilla

#endif  // PERCEPTION__OPENCV_CAMERA_NODE_HPP_
