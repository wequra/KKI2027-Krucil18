#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <rclcpp/rclcpp.hpp>

namespace dagozilla
{

class CmdVelToRcOverrideNode : public rclcpp::Node
{
public:
  CmdVelToRcOverrideNode()
  : rclcpp::Node("cmd_vel_to_rc_override_node")
  {
    enabled_ = this->declare_parameter<bool>("enabled", true);
    steering_channel_ = this->declare_parameter<int>("steering_channel", 1);
    throttle_channel_ = this->declare_parameter<int>("throttle_channel", 3);
    pwm_min_ = this->declare_parameter<int>("pwm_min", 1200);
    pwm_trim_ = this->declare_parameter<int>("pwm_trim", 1500);
    pwm_max_ = this->declare_parameter<int>("pwm_max", 1800);
    max_linear_input_ = this->declare_parameter<double>("max_linear_input", 1.0);
    max_angular_input_ = this->declare_parameter<double>("max_angular_input", 1.0);
    deadman_timeout_ = this->declare_parameter<double>("deadman_timeout", 0.75);
    publish_rate_ = this->declare_parameter<double>("publish_rate", 10.0);

    validateParameters();

    command_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/asv/cmd_vel", 10,
      std::bind(&CmdVelToRcOverrideNode::commandCallback, this, std::placeholders::_1));
    override_publisher_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>(
      "/mavros/rc/override", 10);

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / publish_rate_));
    publish_timer_ = this->create_wall_timer(
      period, std::bind(&CmdVelToRcOverrideNode::publishOverride, this));

    RCLCPP_INFO(
      this->get_logger(),
      "RC override backend %s: steering channel %d, throttle channel %d, "
      "PWM %d/%d/%d, %.1f Hz, deadman %.2f s.",
      enabled_ ? "enabled" : "disabled", steering_channel_, throttle_channel_,
      pwm_min_, pwm_trim_, pwm_max_, publish_rate_, deadman_timeout_);
  }

private:
  static constexpr int kChannelCount = 18;

  void validateParameters() const
  {
    const auto valid_channel = [](int channel) {
        return channel >= 1 && channel <= kChannelCount;
      };

    if (!valid_channel(steering_channel_) || !valid_channel(throttle_channel_)) {
      throw std::invalid_argument("steering_channel and throttle_channel must be in [1, 18]");
    }
    if (steering_channel_ == throttle_channel_) {
      throw std::invalid_argument("steering_channel and throttle_channel must be different");
    }
    if (!(pwm_min_ < pwm_trim_ && pwm_trim_ < pwm_max_) ||
      pwm_min_ < 1 || pwm_max_ > 65534)
    {
      throw std::invalid_argument("PWM parameters must satisfy 1 <= min < trim < max <= 65534");
    }
    if (!std::isfinite(max_linear_input_) || max_linear_input_ <= 0.0 ||
      !std::isfinite(max_angular_input_) || max_angular_input_ <= 0.0)
    {
      throw std::invalid_argument("max input parameters must be finite and greater than zero");
    }
    if (!std::isfinite(deadman_timeout_) || deadman_timeout_ <= 0.0 ||
      !std::isfinite(publish_rate_) || publish_rate_ <= 0.0)
    {
      throw std::invalid_argument("deadman_timeout and publish_rate must be finite and positive");
    }
  }

  void commandCallback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    if (!enabled_) {
      return;
    }

    if (!std::isfinite(message->linear.x) || !std::isfinite(message->angular.z)) {
      has_command_ = false;
      RCLCPP_WARN(this->get_logger(), "Ignoring non-finite /asv/cmd_vel; output set to neutral.");
      return;
    }

    last_command_ = *message;
    last_command_time_ = std::chrono::steady_clock::now();
    has_command_ = true;
  }

  uint16_t mapInputToPwm(double input, double maximum_input) const
  {
    const double normalized = std::clamp(input / maximum_input, -1.0, 1.0);
    const double pwm = normalized >= 0.0 ?
      pwm_trim_ + normalized * (pwm_max_ - pwm_trim_) :
      pwm_trim_ + normalized * (pwm_trim_ - pwm_min_);
    return static_cast<uint16_t>(std::lround(pwm));
  }

  void publishOverride()
  {
    if (!enabled_) {
      return;
    }

    mavros_msgs::msg::OverrideRCIn output;
    output.channels.fill(mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE);

    uint16_t steering_pwm = static_cast<uint16_t>(pwm_trim_);
    uint16_t throttle_pwm = static_cast<uint16_t>(pwm_trim_);
    if (has_command_) {
      const double command_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_command_time_).count();
      if (command_age <= deadman_timeout_) {
        steering_pwm = mapInputToPwm(-last_command_.angular.z, max_angular_input_);
        throttle_pwm = mapInputToPwm(last_command_.linear.x, max_linear_input_);
      }
    }

    output.channels.at(static_cast<size_t>(steering_channel_ - 1)) = steering_pwm;
    output.channels.at(static_cast<size_t>(throttle_channel_ - 1)) = throttle_pwm;
    override_publisher_->publish(output);
  }

  bool enabled_{true};
  int steering_channel_{1};
  int throttle_channel_{3};
  int pwm_min_{1200};
  int pwm_trim_{1500};
  int pwm_max_{1800};
  double max_linear_input_{1.0};
  double max_angular_input_{1.0};
  double deadman_timeout_{0.75};
  double publish_rate_{10.0};

  bool has_command_{false};
  geometry_msgs::msg::Twist last_command_{};
  std::chrono::steady_clock::time_point last_command_time_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr override_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dagozilla::CmdVelToRcOverrideNode>());
  rclcpp::shutdown();
  return 0;
}
