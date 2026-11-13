#ifndef PERCEPTION__HSV_TUNER_NODE_HPP_
#define PERCEPTION__HSV_TUNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

namespace dagozilla
{

class HsvTunerNode : public rclcpp::Node
{
public:
  explicit HsvTunerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HsvTunerNode() override;

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  void handleKeypress(int key);
  void printYamlConfig();
  void setPreset(const std::string & name, int hl, int hh, int sl, int sh, int vl, int vh);
  void updateTrackbarPositions();

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;

  int h_low_{0}, s_low_{100}, v_low_{100};
  int h_high_{179}, s_high_{255}, v_high_{255};
  bool show_gui_{true};
};

}  // namespace dagozilla

#endif  // PERCEPTION__HSV_TUNER_NODE_HPP_
