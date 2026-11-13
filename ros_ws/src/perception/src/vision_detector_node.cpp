#include "perception/vision_detector_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <functional>

namespace dagozilla
{

VisionDetectorNode::VisionDetectorNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vision_detector_node", options)
{
  targets_.push_back(loadTarget(
    "red_low", msgs::msg::ObstacleArray::RED_BUOY,
    cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), 300.0));
  targets_.push_back(loadTarget(
    "red_high", msgs::msg::ObstacleArray::RED_BUOY,
    cv::Scalar(170, 120, 70), cv::Scalar(179, 255, 255), 300.0));
  targets_.push_back(loadTarget(
    "green", msgs::msg::ObstacleArray::GREEN_BUOY,
    cv::Scalar(35, 80, 60), cv::Scalar(85, 255, 255), 300.0));
  targets_.push_back(loadTarget(
    "blue", msgs::msg::ObstacleArray::BLUE_DOCKING_BUOY,
    cv::Scalar(95, 80, 50), cv::Scalar(130, 255, 255), 300.0));

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&VisionDetectorNode::imageCallback, this, std::placeholders::_1));

  obstacle_pub_ = this->create_publisher<msgs::msg::ObstacleArray>(
    "obstacles", 10);
  debug_image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/asv/camera_debug", 10);

  RCLCPP_INFO(
    this->get_logger(),
    "vision_detector_node siap: deteksi buoy merah-hijau dan marker/docking biru KKI 2026.");
}

void VisionDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat debug_frame = cv_ptr->image.clone();
  auto obstacles = detect(cv_ptr->image, msg->header);

  // Draw detected obstacles on debug_frame
  for (size_t i = 0; i < obstacles.positions.size(); ++i) {
    cv::Point2f center(obstacles.positions[i].x, obstacles.positions[i].y);
    float radius = obstacles.radii[i];
    uint8_t cls = obstacles.classes[i];

    cv::Scalar color(255, 255, 255);
    std::string label = "OBJ";
    if (cls == msgs::msg::ObstacleArray::RED_BUOY) {
      color = cv::Scalar(0, 0, 255);
      label = "RED_BUOY";
    } else if (cls == msgs::msg::ObstacleArray::GREEN_BUOY) {
      color = cv::Scalar(0, 255, 0);
      label = "GREEN_BUOY";
    } else if (cls == msgs::msg::ObstacleArray::BLUE_DOCKING_BUOY) {
      color = cv::Scalar(255, 0, 0);
      label = "BLUE_BUOY";
    }

    cv::circle(debug_frame, center, static_cast<int>(radius), color, 2);
    cv::circle(debug_frame, center, 4, color, -1);
    cv::putText(debug_frame, label, cv::Point(center.x - 20, center.y - radius - 5),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 2);
  }

  // Publish debug image for live stream / GCS viewer
  cv_bridge::CvImage debug_msg;
  debug_msg.header = msg->header;
  debug_msg.encoding = "bgr8";
  debug_msg.image = debug_frame;
  debug_image_pub_->publish(*debug_msg.toImageMsg());

  obstacle_pub_->publish(obstacles);
}

VisionDetectorNode::ColorTarget VisionDetectorNode::loadTarget(
  const std::string & prefix,
  uint8_t obstacle_class,
  const cv::Scalar & default_lower,
  const cv::Scalar & default_upper,
  double default_min_area)
{
  const auto h_low =
    this->declare_parameter<int>(prefix + ".h_low", static_cast<int>(default_lower[0]));
  const auto s_low =
    this->declare_parameter<int>(prefix + ".s_low", static_cast<int>(default_lower[1]));
  const auto v_low =
    this->declare_parameter<int>(prefix + ".v_low", static_cast<int>(default_lower[2]));
  const auto h_high =
    this->declare_parameter<int>(prefix + ".h_high", static_cast<int>(default_upper[0]));
  const auto s_high =
    this->declare_parameter<int>(prefix + ".s_high", static_cast<int>(default_upper[1]));
  const auto v_high =
    this->declare_parameter<int>(prefix + ".v_high", static_cast<int>(default_upper[2]));
  const auto min_area =
    this->declare_parameter<double>(prefix + ".min_contour_area", default_min_area);

  return ColorTarget{
    prefix,
    obstacle_class,
    cv::Scalar(h_low, s_low, v_low),
    cv::Scalar(h_high, s_high, v_high),
    min_area};
}

void VisionDetectorNode::detectTarget(
  const cv::Mat & hsv,
  const ColorTarget & target,
  msgs::msg::ObstacleArray & result)
{
  cv::Mat mask;
  cv::inRange(hsv, target.lower, target.upper, mask);

  cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
  cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < target.min_area) {
      continue;
    }

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);
    const double circle_area = std::max(1.0, 3.14159265358979323846 * radius * radius);
    const auto confidence = static_cast<float>(std::clamp(area / circle_area, 0.0, 1.0));

    geometry_msgs::msg::Point p;
    p.x = center.x;
    p.y = center.y;
    p.z = 0.0;

    result.positions.push_back(p);
    result.radii.push_back(radius);
    result.classes.push_back(target.obstacle_class);
    result.confidences.push_back(confidence);
  }
}

msgs::msg::ObstacleArray VisionDetectorNode::detect(
  const cv::Mat & frame, const std_msgs::msg::Header & header)
{
  msgs::msg::ObstacleArray result;
  result.header = header;

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  for (const auto & target : targets_) {
    detectTarget(hsv, target, result);
  }

  return result;
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::VisionDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
