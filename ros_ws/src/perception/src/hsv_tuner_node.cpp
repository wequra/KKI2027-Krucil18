#include "perception/hsv_tuner_node.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <iostream>

namespace dagozilla
{

HsvTunerNode::HsvTunerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("hsv_tuner_node", options)
{
  this->declare_parameter<bool>("show_gui", true);
  this->declare_parameter<int>("h_low", 0);
  this->declare_parameter<int>("s_low", 100);
  this->declare_parameter<int>("v_low", 100);
  this->declare_parameter<int>("h_high", 179);
  this->declare_parameter<int>("s_high", 255);
  this->declare_parameter<int>("v_high", 255);

  show_gui_ = this->get_parameter("show_gui").as_bool();
  h_low_ = this->get_parameter("h_low").as_int();
  s_low_ = this->get_parameter("s_low").as_int();
  v_low_ = this->get_parameter("v_low").as_int();
  h_high_ = this->get_parameter("h_high").as_int();
  s_high_ = this->get_parameter("s_high").as_int();
  v_high_ = this->get_parameter("v_high").as_int();

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "camera/image_raw", rclcpp::SensorDataQoS(),
    std::bind(&HsvTunerNode::imageCallback, this, std::placeholders::_1));

  mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/asv/hsv_mask", 10);
  result_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/asv/hsv_result", 10);

  if (show_gui_) {
    try {
      cv::namedWindow("HSV Tuner Controls", cv::WINDOW_AUTOSIZE);
      cv::createTrackbar("H Low", "HSV Tuner Controls", &h_low_, 179);
      cv::createTrackbar("H High", "HSV Tuner Controls", &h_high_, 179);
      cv::createTrackbar("S Low", "HSV Tuner Controls", &s_low_, 255);
      cv::createTrackbar("S High", "HSV Tuner Controls", &s_high_, 255);
      cv::createTrackbar("V Low", "HSV Tuner Controls", &v_low_, 255);
      cv::createTrackbar("V High", "HSV Tuner Controls", &v_high_, 255);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV GUI Window not available: %s", e.what());
      show_gui_ = false;
    }
  }

  RCLCPP_INFO(this->get_logger(), "==========================================================");
  RCLCPP_INFO(this->get_logger(), "hsv_tuner_node started. Subscribe 'camera/image_raw'.");
  RCLCPP_INFO(this->get_logger(), "HOTKEYS inside OpenCV GUI window:");
  RCLCPP_INFO(this->get_logger(), "  's' or 'p': Export current HSV settings to ROS 2 YAML format");
  RCLCPP_INFO(this->get_logger(), "  'r'       : Load Red Buoy preset [0-10, 120-255, 70-255]");
  RCLCPP_INFO(this->get_logger(), "  'g'       : Load Green Buoy preset [35-85, 80-255, 60-255]");
  RCLCPP_INFO(this->get_logger(), "  'b'       : Load Blue Dock preset [95-130, 80-255, 50-255]");
  RCLCPP_INFO(this->get_logger(), "==========================================================");
}

HsvTunerNode::~HsvTunerNode()
{
  if (show_gui_) {
    cv::destroyAllWindows();
  }
}

void HsvTunerNode::setPreset(const std::string & name, int hl, int hh, int sl, int sh, int vl, int vh)
{
  h_low_ = hl; h_high_ = hh;
  s_low_ = sl; s_high_ = sh;
  v_low_ = vl; v_high_ = vh;
  updateTrackbarPositions();
  RCLCPP_INFO(this->get_logger(), "Loaded Preset [%s]: H:[%d-%d] S:[%d-%d] V:[%d-%d]",
              name.c_str(), h_low_, h_high_, s_low_, s_high_, v_low_, v_high_);
}

void HsvTunerNode::updateTrackbarPositions()
{
  if (!show_gui_) return;
  cv::setTrackbarPos("H Low", "HSV Tuner Controls", h_low_);
  cv::setTrackbarPos("H High", "HSV Tuner Controls", h_high_);
  cv::setTrackbarPos("S Low", "HSV Tuner Controls", s_low_);
  cv::setTrackbarPos("S High", "HSV Tuner Controls", s_high_);
  cv::setTrackbarPos("V Low", "HSV Tuner Controls", v_low_);
  cv::setTrackbarPos("V High", "HSV Tuner Controls", v_high_);
}

void HsvTunerNode::printYamlConfig()
{
  std::cout << "\n==========================================================\n";
  std::cout << "### CURRENT HSV TUNER VALUES (YAML Format for vision_params.yaml) ###\n";
  std::cout << "  h_low: " << h_low_ << "\n";
  std::cout << "  h_high: " << h_high_ << "\n";
  std::cout << "  s_low: " << s_low_ << "\n";
  std::cout << "  s_high: " << s_high_ << "\n";
  std::cout << "  v_low: " << v_low_ << "\n";
  std::cout << "  v_high: " << v_high_ << "\n";
  std::cout << "==========================================================\n" << std::endl;
}

void HsvTunerNode::handleKeypress(int key)
{
  if (key < 0) return;
  char c = static_cast<char>(key & 0xFF);
  if (c == 's' || c == 'S' || c == 'p' || c == 'P') {
    printYamlConfig();
  } else if (c == 'r' || c == 'R') {
    setPreset("Red Buoy", 0, 10, 120, 255, 70, 255);
  } else if (c == 'g' || c == 'G') {
    setPreset("Green Buoy", 35, 85, 80, 255, 60, 255);
  } else if (c == 'b' || c == 'B') {
    setPreset("Blue Dock", 95, 130, 80, 255, 50, 255);
  }
}

void HsvTunerNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv, mask, result;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

  cv::Scalar lower(h_low_, s_low_, v_low_);
  cv::Scalar upper(h_high_, s_high_, v_high_);
  cv::inRange(hsv, lower, upper, mask);
  cv::bitwise_and(cv_ptr->image, cv_ptr->image, result, mask);

  if (show_gui_) {
    // Add text overlay legend on result frame
    cv::Mat gui_result = result.clone();
    std::string hsv_str = cv::format("H:[%d-%d] S:[%d-%d] V:[%d-%d]",
                                     h_low_, h_high_, s_low_, s_high_, v_low_, v_high_);
    cv::putText(gui_result, hsv_str, cv::Point(10, 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
    cv::putText(gui_result, "Hotkeys: [S] Dump YAML | [R] Red | [G] Green | [B] Blue",
                cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    cv::imshow("Mask", mask);
    cv::imshow("Result", gui_result);
    int key = cv::waitKey(1);
    handleKeypress(key);
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 5000,
    "Current HSV Threshold: H:[%d - %d], S:[%d - %d], V:[%d - %d]",
    h_low_, h_high_, s_low_, s_high_, v_low_, v_high_);

  cv_bridge::CvImage mask_msg(msg->header, "mono8", mask);
  cv_bridge::CvImage result_msg(msg->header, "bgr8", result);
  mask_pub_->publish(*mask_msg.toImageMsg());
  result_pub_->publish(*result_msg.toImageMsg());
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::HsvTunerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
