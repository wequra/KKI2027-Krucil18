#include "mission/mission_manager_node.hpp"
#include <chrono>
#include <algorithm>
#include <functional>

using namespace std::chrono_literals;

namespace dagozilla
{

MissionManagerNode::MissionManagerNode()
: rclcpp::Node("mission_manager_node")
{
  total_buoy_pairs_ =
    static_cast<uint32_t>(this->declare_parameter<int>("total_buoy_pairs", 10));
  max_run_seconds_ =
    static_cast<uint32_t>(this->declare_parameter<int>("max_run_seconds", 20 * 60));

  wp_pub_ = this->create_publisher<msgs::msg::Waypoint>("/asv/active_waypoint", 10);
  status_pub_ = this->create_publisher<msgs::msg::MissionStatus>("/asv/mission_status", 10);
  gate_count_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/asv/buoy_gate_count", 10,
    std::bind(&MissionManagerNode::onGateCount, this, std::placeholders::_1));

  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/start_mission",
    std::bind(&MissionManagerNode::onStart, this, std::placeholders::_1, std::placeholders::_2));

  abort_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/abort_mission",
    std::bind(&MissionManagerNode::onAbort, this, std::placeholders::_1, std::placeholders::_2));

  complete_buoy_pair_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/complete_buoy_pair",
    std::bind(
      &MissionManagerNode::onCompleteBuoyPair, this,
      std::placeholders::_1, std::placeholders::_2));

  surface_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/mark_surface_image",
    std::bind(&MissionManagerNode::onSurfaceImage, this, std::placeholders::_1, std::placeholders::_2));

  underwater_image_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/mark_underwater_image",
    std::bind(
      &MissionManagerNode::onUnderwaterImage, this,
      std::placeholders::_1, std::placeholders::_2));

  docking_complete_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/complete_docking",
    std::bind(
      &MissionManagerNode::onDockingComplete, this,
      std::placeholders::_1, std::placeholders::_2));

  penalty_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/asv/record_penalty",
    std::bind(&MissionManagerNode::onPenalty, this, std::placeholders::_1, std::placeholders::_2));

  status_timer_ = this->create_wall_timer(1s, [this]() {
    checkTimeout();
    publishStatus();
  });

  RCLCPP_INFO(
    this->get_logger(),
    "mission_manager_node siap untuk urutan ASV KKI 2026: start, buoy, imaging, docking, finish.");
}

void MissionManagerNode::loadWaypoints(const std::vector<msgs::msg::Waypoint> & waypoints)
{
  waypoints_ = waypoints;
  current_index_ = 0;
}

void MissionManagerNode::onStart(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  resetRunProgress();
  state_ = msgs::msg::MissionStatus::NAVIGATING_BUOYS;
  run_started_at_ = this->get_clock()->now();
  publishActiveWaypoint();
  response->success = true;
  response->message = "Run dimulai: navigasi 10 pasang buoy merah-hijau.";
}

void MissionManagerNode::onAbort(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  state_ = msgs::msg::MissionStatus::ABORTED;
  response->success = true;
  response->message = "Misi dibatalkan.";
}

void MissionManagerNode::onCompleteBuoyPair(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_ != msgs::msg::MissionStatus::NAVIGATING_BUOYS) {
    response->success = false;
    response->message = "State bukan NAVIGATING_BUOYS.";
    return;
  }

  buoy_pairs_passed_ = std::min(buoy_pairs_passed_ + 1, total_buoy_pairs_);
  current_index_ = std::min<std::size_t>(current_index_ + 1, waypoints_.size());

  if (buoy_pairs_passed_ >= total_buoy_pairs_) {
    state_ = msgs::msg::MissionStatus::UNDERWATER_IMAGING;
    response->message = "Semua pasangan buoy selesai. Lanjut underwater imaging kotak biru.";
  } else {
    publishActiveWaypoint();
    response->message = "Pasangan buoy tercatat.";
  }

  response->success = true;
}

void MissionManagerNode::onSurfaceImage(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_ != msgs::msg::MissionStatus::SURFACE_IMAGING) {
    response->success = false;
    response->message = "State bukan SURFACE_IMAGING.";
    return;
  }

  surface_image_captured_ = true;
  state_ = msgs::msg::MissionStatus::DOCKING;
  response->success = true;
  response->message = "Surface imaging tercatat. Lanjut docking ke buoy biru.";
}

void MissionManagerNode::onUnderwaterImage(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_ != msgs::msg::MissionStatus::UNDERWATER_IMAGING) {
    response->success = false;
    response->message = "State bukan UNDERWATER_IMAGING.";
    return;
  }

  underwater_image_captured_ = true;
  state_ = msgs::msg::MissionStatus::SURFACE_IMAGING;
  response->success = true;
  response->message = "Underwater imaging tercatat. Lanjut surface imaging kotak hijau.";
}

void MissionManagerNode::onGateCount(const std_msgs::msg::UInt8::SharedPtr message)
{
  if (state_ != msgs::msg::MissionStatus::NAVIGATING_BUOYS) return;
  buoy_pairs_passed_ = std::min<uint32_t>(message->data, total_buoy_pairs_);
  if (buoy_pairs_passed_ >= total_buoy_pairs_) {
    state_ = msgs::msg::MissionStatus::UNDERWATER_IMAGING;
    RCLCPP_INFO(this->get_logger(), "Gate %u selesai: mulai UNDERWATER_IMAGING.", total_buoy_pairs_);
  }
}

void MissionManagerNode::onDockingComplete(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (state_ != msgs::msg::MissionStatus::DOCKING) {
    response->success = false;
    response->message = "State bukan DOCKING.";
    return;
  }

  docking_completed_ = true;
  state_ = msgs::msg::MissionStatus::COMPLETED;
  response->success = true;
  response->message = "Docking tercatat. Run selesai.";
}

void MissionManagerNode::onPenalty(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!missionActive()) {
    response->success = false;
    response->message = "Tidak ada run aktif.";
    return;
  }

  penalty_count_++;
  if (penalty_count_ > 5) {
    resetRunProgress();
    state_ = msgs::msg::MissionStatus::NAVIGATING_BUOYS;
    run_started_at_ = this->get_clock()->now();
    response->message = "Penalti > 5. Run diulang dari START sesuai panduan.";
  } else {
    response->message = "Penalti tercatat.";
  }

  response->success = true;
}

void MissionManagerNode::publishActiveWaypoint()
{
  if (current_index_ < waypoints_.size()) {
    wp_pub_->publish(waypoints_[current_index_]);
  }
}

void MissionManagerNode::publishStatus()
{
  msgs::msg::MissionStatus msg;
  msg.state = state_;
  msg.current_waypoint_index = static_cast<uint32_t>(current_index_);
  msg.total_waypoints = static_cast<uint32_t>(waypoints_.size());
  msg.buoy_pairs_passed = buoy_pairs_passed_;
  msg.total_buoy_pairs = total_buoy_pairs_;
  msg.penalty_count = penalty_count_;
  msg.elapsed_seconds = elapsedSeconds();
  msg.max_run_seconds = max_run_seconds_;
  msg.surface_image_captured = surface_image_captured_;
  msg.underwater_image_captured = underwater_image_captured_;
  msg.docking_completed = docking_completed_;
  status_pub_->publish(msg);
}

void MissionManagerNode::checkTimeout()
{
  if (missionActive() && elapsedSeconds() >= max_run_seconds_) {
    state_ = msgs::msg::MissionStatus::TIMEOUT;
    RCLCPP_WARN(this->get_logger(), "Run melewati batas waktu %u detik.", max_run_seconds_);
  }
}

void MissionManagerNode::resetRunProgress()
{
  current_index_ = 0;
  buoy_pairs_passed_ = 0;
  penalty_count_ = 0;
  surface_image_captured_ = false;
  underwater_image_captured_ = false;
  docking_completed_ = false;
}

bool MissionManagerNode::missionActive() const
{
  return state_ == msgs::msg::MissionStatus::NAVIGATING_BUOYS ||
         state_ == msgs::msg::MissionStatus::SURFACE_IMAGING ||
         state_ == msgs::msg::MissionStatus::UNDERWATER_IMAGING ||
         state_ == msgs::msg::MissionStatus::DOCKING;
}

uint32_t MissionManagerNode::elapsedSeconds() const
{
  if (!missionActive() && state_ != msgs::msg::MissionStatus::COMPLETED &&
    state_ != msgs::msg::MissionStatus::TIMEOUT)
  {
    return 0;
  }

  const auto elapsed = this->get_clock()->now() - run_started_at_;
  const auto seconds = elapsed.seconds();
  if (seconds <= 0.0) {
    return 0;
  }
  return static_cast<uint32_t>(seconds);
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dagozilla::MissionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
