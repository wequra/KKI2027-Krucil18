#ifndef MISSION__MISSION_MANAGER_NODE_HPP_
#define MISSION__MISSION_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <msgs/msg/waypoint.hpp>
#include <msgs/msg/mission_status.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <chrono>
#include <vector>

namespace dagozilla
{

/// State machine level tinggi mengikuti urutan misi ASV KKI 2026:
/// start, 10 pasang buoy, surface imaging, underwater imaging, docking, finish.
class MissionManagerNode : public rclcpp::Node
{
public:
  MissionManagerNode();

  void loadWaypoints(const std::vector<msgs::msg::Waypoint> & waypoints);

private:
  void onStart(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onAbort(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onCompleteBuoyPair(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onSurfaceImage(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onUnderwaterImage(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onDockingComplete(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onPenalty(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void onGateCount(const std_msgs::msg::UInt8::SharedPtr message);

  void publishActiveWaypoint();
  void publishStatus();
  void checkTimeout();
  void resetRunProgress();
  bool missionActive() const;
  uint32_t elapsedSeconds() const;

  std::vector<msgs::msg::Waypoint> waypoints_;
  std::size_t current_index_{0};
  uint8_t state_{msgs::msg::MissionStatus::IDLE};
  uint32_t buoy_pairs_passed_{0};
  uint32_t total_buoy_pairs_{10};
  uint32_t penalty_count_{0};
  uint32_t max_run_seconds_{20 * 60};
  bool surface_image_captured_{false};
  bool underwater_image_captured_{false};
  bool docking_completed_{false};
  rclcpp::Time run_started_at_;

  rclcpp::Publisher<msgs::msg::Waypoint>::SharedPtr wp_pub_;
  rclcpp::Publisher<msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gate_count_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr complete_buoy_pair_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr surface_image_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr underwater_image_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr docking_complete_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr penalty_srv_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace dagozilla

#endif  // MISSION__MISSION_MANAGER_NODE_HPP_
