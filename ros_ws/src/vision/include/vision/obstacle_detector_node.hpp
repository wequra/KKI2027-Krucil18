#ifndef VISION__OBSTACLE_DETECTOR_NODE_HPP_
#define VISION__OBSTACLE_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <msgs/msg/obstacle_array.hpp>

namespace dagozilla
{

/// Stub deteksi obstacle generik (mis. dari lidar). Isi dengan clustering
/// atau algoritma sensor lain di luar vision (lihat perception
/// untuk pipeline kamera).
class ObstacleDetectorNode : public rclcpp::Node
{
public:
  ObstacleDetectorNode();

private:
  void detectLoop();
  rclcpp::Publisher<msgs::msg::ObstacleArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dagozilla

#endif  // VISION__OBSTACLE_DETECTOR_NODE_HPP_
