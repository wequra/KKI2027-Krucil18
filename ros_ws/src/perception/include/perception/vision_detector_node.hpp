#ifndef PERCEPTION__VISION_DETECTOR_NODE_HPP_
#define PERCEPTION__VISION_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <msgs/msg/obstacle_array.hpp>
#include <string>
#include <vector>

namespace dagozilla
{

/// Node deteksi obstacle/buoy dari kamera menggunakan color thresholding.
/// Ganti bagian detect() dengan pipeline yang lebih canggih (YOLO/DNN) kalau perlu.
class VisionDetectorNode : public rclcpp::Node
{
public:
  explicit VisionDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct ColorTarget
  {
    std::string name;
    uint8_t obstacle_class;
    cv::Scalar lower;
    cv::Scalar upper;
    double min_area;
  };

  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg);
  msgs::msg::ObstacleArray detect(const cv::Mat & frame, const std_msgs::msg::Header & header);
  void detectTarget(
    const cv::Mat & hsv,
    const ColorTarget & target,
    msgs::msg::ObstacleArray & result);
  ColorTarget loadTarget(
    const std::string & prefix,
    uint8_t obstacle_class,
    const cv::Scalar & default_lower,
    const cv::Scalar & default_upper,
    double default_min_area);

  std::vector<ColorTarget> targets_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_image_pub_;
};

}  // namespace dagozilla

#endif  // PERCEPTION__VISION_DETECTOR_NODE_HPP_
