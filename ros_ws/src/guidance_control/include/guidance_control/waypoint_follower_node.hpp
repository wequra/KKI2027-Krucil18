#ifndef GUIDANCE_CONTROL__WAYPOINT_FOLLOWER_NODE_HPP_
#define GUIDANCE_CONTROL__WAYPOINT_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <msgs/msg/waypoint.hpp>
#include <msgs/msg/mission_status.hpp>
#include <optional>

namespace dagozilla
{

/// Guidance & control sederhana: bergerak menuju satu waypoint aktif,
/// publish setpoint kecepatan ke MAVROS (mode GUIDED di ArduRover).
/// SKELETON — ganti bagian kontrol dengan algoritma tim (PID heading,
/// pure pursuit, LOS guidance, dst).
class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode();

private:
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void waypointCallback(const msgs::msg::Waypoint::SharedPtr msg);
  void headingCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void controlLoop();
  void publishCmd(double linear_x, double angular_z);

  static void bearingDistance(
    double lat1, double lon1, double lat2, double lon2,
    double & bearing_deg, double & distance_m);

  std::optional<msgs::msg::Waypoint> active_waypoint_;
  sensor_msgs::msg::NavSatFix current_gps_;
  bool gps_received_{false};
  
  double current_heading_{0.0};
  bool heading_received_{false};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<msgs::msg::Waypoint>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace dagozilla

#endif  // GUIDANCE_CONTROL__WAYPOINT_FOLLOWER_NODE_HPP_
