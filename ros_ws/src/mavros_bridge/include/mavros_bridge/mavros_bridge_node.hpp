#ifndef MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_
#define MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_

#include <chrono>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <msgs/srv/set_mode.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#ifdef HAVE_MAVROS_MSGS
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#endif

namespace dagozilla
{

/// Jembatan antara stack ROS2 (mission/guidance) dengan Pixhawk (ArduRover)
/// lewat MAVROS: baca state & GPS, sediakan service ganti flight mode, relay cmd_vel.
class MavrosBridgeNode : public rclcpp::Node
{
public:
  MavrosBridgeNode();

private:
#ifdef HAVE_MAVROS_MSGS
  void stateCallback(const mavros_msgs::msg::State::SharedPtr msg);
#endif
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishCommand();
  void configureMavrosVelocityFrame();
  void onSetMode(
    const std::shared_ptr<msgs::srv::SetMode::Request> request,
    std::shared_ptr<msgs::srv::SetMode::Response> response);

#ifdef HAVE_MAVROS_MSGS
  mavros_msgs::msg::State current_state_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_client_;
#endif
  sensor_msgs::msg::NavSatFix current_gps_;

  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mavros_cmd_vel_pub_;
  rclcpp::Service<msgs::srv::SetMode>::SharedPtr set_mode_srv_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::AsyncParametersClient::SharedPtr mavros_velocity_params_;
  rclcpp::TimerBase::SharedPtr mavros_frame_timer_;
  geometry_msgs::msg::Twist last_cmd_vel_{};
  std::chrono::steady_clock::time_point last_cmd_time_{};
  bool has_command_{false};
  bool mavros_frame_request_in_flight_{false};
  bool mavros_frame_configured_{false};
  double command_timeout_sec_{0.75};
  std::string mavros_velocity_frame_{"BODY_NED"};
};

}  // namespace dagozilla

#endif  // MAVROS_BRIDGE__MAVROS_BRIDGE_NODE_HPP_
