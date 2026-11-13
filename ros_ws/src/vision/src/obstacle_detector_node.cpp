#include "vision/obstacle_detector_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

ObstacleDetectorNode::ObstacleDetectorNode()
: rclcpp::Node("obstacle_detector_node")
{
  pub_ = this->create_publisher<msgs::msg::ObstacleArray>("/asv/obstacles", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&ObstacleDetectorNode::detectLoop, this));
  RCLCPP_INFO(
    this->get_logger(),
    "obstacle_detector_node siap (stub — isi pipeline deteksi lidar/sensor di sini).");
}

void ObstacleDetectorNode::detectLoop()
{
  msgs::msg::ObstacleArray msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_link";
  // TODO: isi msg.positions dan msg.radii dari hasil deteksi lidar/sensor lain
  pub_->publish(msg);
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dagozilla::ObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
