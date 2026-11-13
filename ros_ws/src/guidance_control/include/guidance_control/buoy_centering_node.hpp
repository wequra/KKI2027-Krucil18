#ifndef GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_
#define GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <msgs/msg/obstacle_array.hpp>
#include <msgs/msg/tracked_obstacle_array.hpp>
#include <std_msgs/msg/string.hpp>
#include "guidance_control/gate_logic.hpp"
#include <msgs/msg/mission_status.hpp>
#include <msgs/msg/controller_diagnostic.hpp>
#include "guidance_control/steering_pid.hpp"

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

namespace dagozilla
{

/// Node guidance_control untuk melewati rangkaian gate buoy merah-hijau.
/// Controller ini sengaja berdiri sendiri untuk uji lapangan: tidak bergantung
/// pada mission_manager_node atau waypoint_follower_node.
class BuoyCenteringNode : public rclcpp::Node
{
public:
  BuoyCenteringNode();
  ~BuoyCenteringNode() override;

private:
  enum class ControllerState
  {
    SEARCHING,
    APPROACHING,
    PASSING,
    CLEARING_GATE,
    TURNING,
    TURN_STABILIZING,
    FINISHED
  };

  struct PairDetection
  {
    bool valid{false};
    int64_t red_id{-1}, green_id{-1};
    std::string session;
    size_t red_index{0};
    size_t green_index{0};
    double red_x{-1.0};
    double red_y{-1.0};
    double green_x{-1.0};
    double green_y{-1.0};
    float red_radius{0.0F};
    float green_radius{0.0F};
    float red_confidence{0.0F};
    float green_confidence{0.0F};
  };

  struct BuoyObservation
  {
    bool valid{false};
    int64_t id{-1};
    size_t index{0};
    double x{-1.0};
    double y{-1.0};
    float radius{0.0F};
    float confidence{0.0F};
  };

  void obstacleCallback(const msgs::msg::ObstacleArray::SharedPtr msg);
  void trackedCallback(const msgs::msg::TrackedObstacleArray::SharedPtr msg);
  void controlTick();
  bool maneuverActive() const;
  bool retired(const PairDetection & pair) const;
  void publishTrackingDebug(size_t count);
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void detectorDebugImageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void missionCallback(const msgs::msg::MissionStatus::SharedPtr msg);
  void headingCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg);
  std::vector<PairDetection> extractValidPairs(const msgs::msg::ObstacleArray & msg) const;
  std::vector<PairDetection> extractAssignedPairs(const msgs::msg::ObstacleArray & msg) const;
  PairDetection extractPair(
    const msgs::msg::ObstacleArray & msg,
    const PairDetection * excluded_target = nullptr) const;
  PairDetection findAssociatedPair(
    const msgs::msg::ObstacleArray & msg,
    const PairDetection & target) const;
  bool findObservation(
    const msgs::msg::ObstacleArray & msg, uint8_t buoy_class, int64_t id,
    BuoyObservation & observation) const;
  bool tryReassociate(
    const msgs::msg::ObstacleArray & msg, const PairDetection & target,
    const BuoyObservation & red, const BuoyObservation & green,
    PairDetection & recovered);
  bool matchesTarget(const PairDetection & candidate, const PairDetection & target) const;
  double targetScore(const PairDetection & pair) const;
  bool considerTargetSwitch(
    const msgs::msg::ObstacleArray & msg, const PairDetection & current,
    PairDetection & selected);
  void resetSwitchCandidate(const std::string & reason);
  void clearCurrentTarget();
  void updateRecentlyPassedTarget(const msgs::msg::ObstacleArray & msg);
  void handleSearching();
  std::pair<double, double> searchingCommand() const;
  void handleApproaching(const PairDetection & pair);
  void handlePartialRecovery(
    const BuoyObservation & visible, bool missing_red, const PairDetection & target);
  void handlePassing();
  void markGatePassed();
  void transitionTo(ControllerState next_state);
  const char * stateName() const;
  const char * stateName(ControllerState state) const;
  const char * missionStateName(uint8_t state) const;
  bool isNearGate(const PairDetection & pair) const;
  void publishGateCount();
  void publishCmd(double linear_x, double angular_z);
  void publishDiagnostic(
    bool target_detected, double error_x, double linear_x, double angular_z,
    double p, double i, double d);
  void renderOverlay(
    cv::Mat & frame,
    const msgs::msg::ObstacleArray::SharedPtr & obs_msg,
    const PairDetection & pair,
    double midpoint_x, double error_x,
    double cmd_linear, double cmd_angular,
    ControllerState state);

  double heading_kp_{1.5};
  double steering_ki_{0.0};
  double steering_kd_{0.0};
  double steering_integral_limit_{1.0};
  double steering_output_limit_{1.0};
  double steering_dt_min_{0.001};
  double steering_dt_max_{1.0};
  double max_linear_speed_{1.0};
  double max_yaw_rate_{1.0};
  double frame_width_{640.0};
  double frame_height_{480.0};
  double speed_reduction_factor_{0.6};
  double search_linear_speed_{0.15};
  std::string course_layout_{"A"};
  double passing_linear_speed_{0.30};
  double pass_radius_threshold_{50.0};
  int lost_frames_for_pass_{8};
  int approaching_lost_frames_tolerance_{6};
  int passed_gate_reacquire_block_frames_{3};
  int total_gates_{10};
  double min_pair_confidence_{0.2};
  double max_radius_ratio_{3.0};
  double max_pair_vertical_offset_ratio_{0.25};
  double min_pair_horizontal_separation_ratio_{0.05};
  double max_pair_horizontal_separation_ratio_{0.80};
  double passing_same_gate_min_radius_ratio_{0.70};
  double target_association_max_position_ratio_{0.25};
  double target_association_max_radius_ratio_{3.0};
  bool show_gui_{true};
  bool use_object_tracking_{true}, debug_pairing_{false};
  int candidate_confirm_frames_{3}, candidate_frames_{0};
  PairDetection pending_candidate_;
  std::vector<int64_t> frame_ids_;
  std::string tracker_session_;
  std::string expected_gate_color_order_{"any"};
  double score_center_weight_{1.0}, score_vertical_weight_{1.0};
  double score_size_weight_{2.0}, score_size_reference_radius_{50.0};
  double score_confidence_weight_{0.2};
  double score_continuity_weight_{0.2};
  double score_vertical_mismatch_weight_{4.0};
  double pairing_width_weight_{1.0}, pairing_vertical_weight_{1.0}, pairing_radius_weight_{1.0};
  double target_switch_score_margin_{0.5};
  double target_switch_min_mean_y_advantage_ratio_{0.08};
  double target_switch_min_radius_ratio_{1.15};
  double target_switch_max_vertical_mismatch_ratio_{0.10};
  int target_switch_confirm_frames_{3};
  int partial_gate_recovery_frames_{12};
  double recovery_linear_speed_{0.15}, recovery_yaw_rate_{0.35};
  double reassociation_max_position_ratio_{0.12};
  double reassociation_max_radius_ratio_{1.8};
  double reassociation_max_vertical_mismatch_ratio_{0.10};
  double last_obstacle_stamp_{-1.0};
  double max_detection_age_s_{1.0};
  double passed_gate_suppression_time_s_{30.0};
  double retired_position_ratio_{0.12}, retired_radius_ratio_{1.8};
  struct Retired {PairDetection pair; double time;};
  std::vector<Retired> retired_gates_;
  double gate_3_clear_time_s_{3.0}, gate_7_clear_time_s_{3.0};
  double clear_forward_speed_{0.3}, clear_heading_kp_{0.02};
  double turn_angle_deg_{90.0}, turn_forward_speed_{0.2}, turn_angular_command_{0.4};
  double turn_heading_tolerance_deg_{5.0}, heading_timeout_s_{1.0};
  double detection_timeout_s_{1.0}, turn_timeout_s_{30.0};
  int turn_stable_samples_{5}, stable_samples_{0};
  double heading_received_{-1.0}, obstacle_received_{-1.0};
  double maneuver_started_{0.0}, clearing_elapsed_{0.0}, tick_time_{0.0};
  double clear_heading_{0.0}, turn_start_heading_{0.0}, turn_target_heading_{0.0};
  bool clear_heading_captured_{false}, maneuver_fault_{false};
  double commanded_linear_{0.0}, commanded_angular_{0.0};
  mutable std::vector<PairDetection> debug_candidates_;
  mutable double selected_score_{0.0};
  int visible_locked_{0};
  bool partial_recovery_{false};
  std::string recovery_missing_{"NONE"};
  PairDetection switch_candidate_;
  int switch_confirm_frames_{0};
  std::string switch_reason_{"none"};
  std::string reassociation_debug_{"none"};

  ControllerState state_{ControllerState::SEARCHING};
  int gate_count_{0};
  int lost_frames_{0};
  int approaching_target_lost_frames_{0};
  int recently_passed_target_block_frames_{0};
  bool near_gate_{false};
  bool has_current_target_{false};
  bool has_recently_passed_target_{false};
  PairDetection current_target_;
  PairDetection passing_pair_;
  PairDetection recently_passed_target_;
  float passing_entry_min_radius_{0.0F};
  uint8_t mission_state_{msgs::msg::MissionStatus::IDLE};
  double heading_deg_{0.0};
  bool has_heading_{false};
  mavros_msgs::msg::State mavros_state_;
  bool has_mavros_state_{false};
  SteeringPid steering_pid_;
  rclcpp::Time last_control_time_;
  double last_p_{0.0};
  double last_i_{0.0};
  double last_d_{0.0};
  double last_angular_z_{0.0};

  cv::Mat latest_frame_;
  std_msgs::msg::Header latest_header_;
  bool has_frame_{false};
  cv::Mat latest_detector_debug_frame_;
  std_msgs::msg::Header latest_detector_debug_header_;
  bool has_detector_debug_frame_{false};

  rclcpp::Subscription<msgs::msg::ObstacleArray>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<msgs::msg::TrackedObstacleArray>::SharedPtr tracked_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr tracking_debug_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr detector_debug_sub_;
  rclcpp::Subscription<msgs::msg::MissionStatus>::SharedPtr mission_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr heading_sub_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr mavros_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<msgs::msg::MissionStatus>::SharedPtr status_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gate_count_pub_;
  rclcpp::Publisher<msgs::msg::ControllerDiagnostic>::SharedPtr diagnostic_pub_;
};

}  // namespace dagozilla

#endif  // GUIDANCE_CONTROL__BUOY_CENTERING_NODE_HPP_
