#include "guidance_control/buoy_centering_node.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <set>

namespace dagozilla
{

BuoyCenteringNode::BuoyCenteringNode()
: rclcpp::Node("buoy_centering_node")
{
  this->declare_parameter<double>("heading_kp", 1.5);
  this->declare_parameter<double>("steering_ki", 0.0);
  this->declare_parameter<double>("steering_kd", 0.0);
  this->declare_parameter<double>("steering_integral_limit", 1.0);
  this->declare_parameter<double>("max_linear_speed", 0.8);  // m/s
  this->declare_parameter<double>("max_yaw_rate", 1.0);     // rad/s
  // A non-positive value inherits max_yaw_rate. This preserves the historical
  // clamp for every existing max_yaw_rate configuration.
  this->declare_parameter<double>("steering_output_limit", -1.0);
  this->declare_parameter<double>("steering_dt_min", 0.001);
  this->declare_parameter<double>("steering_dt_max", 1.0);
  this->declare_parameter<double>("frame_width", 640.0);
  this->declare_parameter<double>("speed_reduction_factor", 0.6);
  this->declare_parameter<double>("search_linear_speed", 0.15);
  this->declare_parameter<std::string>("course_layout", "A");
  this->declare_parameter<double>("passing_linear_speed", 0.30);
  this->declare_parameter<double>("pass_radius_threshold", 50.0);
  this->declare_parameter<int>("lost_frames_for_pass", 8);
  this->declare_parameter<int>("approaching_lost_frames_tolerance", 6);
  this->declare_parameter<int>("passed_gate_reacquire_block_frames", 3);
  this->declare_parameter<int>("total_gates", 10);
  this->declare_parameter<double>("min_pair_confidence", 0.2);
  this->declare_parameter<double>("max_radius_ratio", 3.0);
  this->declare_parameter<double>("max_pair_vertical_offset_ratio", 0.25);
  this->declare_parameter<double>("min_pair_horizontal_separation_ratio", 0.05);
  this->declare_parameter<double>("max_pair_horizontal_separation_ratio", 0.80);
  this->declare_parameter<double>("passing_same_gate_min_radius_ratio", 0.70);
  this->declare_parameter<double>("target_association_max_position_ratio", 0.25);
  this->declare_parameter<double>("target_association_max_radius_ratio", 3.0);
  this->declare_parameter<bool>("show_gui", true);

  heading_kp_ = this->get_parameter("heading_kp").as_double();
  steering_ki_ = this->get_parameter("steering_ki").as_double();
  steering_kd_ = this->get_parameter("steering_kd").as_double();
  steering_integral_limit_ = this->get_parameter("steering_integral_limit").as_double();
  max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
  max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
  steering_output_limit_ = this->get_parameter("steering_output_limit").as_double();
  steering_dt_min_ = this->get_parameter("steering_dt_min").as_double();
  steering_dt_max_ = this->get_parameter("steering_dt_max").as_double();
  frame_width_ = this->get_parameter("frame_width").as_double();
  speed_reduction_factor_ = this->get_parameter("speed_reduction_factor").as_double();
  search_linear_speed_ = this->get_parameter("search_linear_speed").as_double();
  course_layout_ = this->get_parameter("course_layout").as_string();
  if (course_layout_ != "A" && course_layout_ != "B") {
    throw std::invalid_argument("course_layout must be A or B");
  }
  passing_linear_speed_ = this->get_parameter("passing_linear_speed").as_double();
  pass_radius_threshold_ = this->get_parameter("pass_radius_threshold").as_double();
  lost_frames_for_pass_ = std::max(1, static_cast<int>(this->get_parameter("lost_frames_for_pass").as_int()));
  approaching_lost_frames_tolerance_ = std::max(
    1, static_cast<int>(this->get_parameter("approaching_lost_frames_tolerance").as_int()));
  passed_gate_reacquire_block_frames_ = std::max(
    1, static_cast<int>(this->get_parameter("passed_gate_reacquire_block_frames").as_int()));
  total_gates_ = std::max(1, static_cast<int>(this->get_parameter("total_gates").as_int()));
  min_pair_confidence_ = this->get_parameter("min_pair_confidence").as_double();
  max_radius_ratio_ = std::max(1.0, this->get_parameter("max_radius_ratio").as_double());
  max_pair_vertical_offset_ratio_ = std::max(0.0, this->get_parameter("max_pair_vertical_offset_ratio").as_double());
  min_pair_horizontal_separation_ratio_ = std::max(0.0, this->get_parameter("min_pair_horizontal_separation_ratio").as_double());
  max_pair_horizontal_separation_ratio_ = std::max(
    min_pair_horizontal_separation_ratio_,
    this->get_parameter("max_pair_horizontal_separation_ratio").as_double());
  passing_same_gate_min_radius_ratio_ = std::clamp(
    this->get_parameter("passing_same_gate_min_radius_ratio").as_double(), 0.0, 1.0);
  target_association_max_position_ratio_ = std::max(
    0.01, this->get_parameter("target_association_max_position_ratio").as_double());
  target_association_max_radius_ratio_ = std::max(
    1.0, this->get_parameter("target_association_max_radius_ratio").as_double());
  show_gui_ = this->get_parameter("show_gui").as_bool();
  use_object_tracking_ = declare_parameter<bool>("use_object_tracking", true);
  debug_pairing_ = declare_parameter<bool>("debug_pairing", false);
  candidate_confirm_frames_ = std::max(1, static_cast<int>(declare_parameter<int>("candidate_confirm_frames", 3)));
  approaching_lost_frames_tolerance_ = std::max(1, static_cast<int>(declare_parameter<int>(
    "tracked_target_lost_frames", approaching_lost_frames_tolerance_)));
  expected_gate_color_order_ = declare_parameter<std::string>("expected_gate_color_order", "any");
  if (expected_gate_color_order_ != "any" && expected_gate_color_order_ != "red_left" &&
    expected_gate_color_order_ != "green_left") throw std::invalid_argument("invalid gate color order");
  score_center_weight_ = declare_parameter<double>("score_center_weight", 1.0);
  score_vertical_weight_ = declare_parameter<double>("score_vertical_weight", 1.0);
  score_size_weight_ = declare_parameter<double>("score_size_weight", 2.0);
  score_size_reference_radius_ = declare_parameter<double>("score_size_reference_radius", 50.0);
  score_confidence_weight_ = declare_parameter<double>("score_confidence_weight", 0.2);
  score_continuity_weight_ = declare_parameter<double>("score_continuity_weight", 0.2);
  score_vertical_mismatch_weight_ = declare_parameter<double>("score_vertical_mismatch_weight", 4.0);
  pairing_width_weight_ = declare_parameter<double>("pairing_width_weight", 1.0);
  pairing_vertical_weight_ = declare_parameter<double>("pairing_vertical_weight", 1.0);
  pairing_radius_weight_ = declare_parameter<double>("pairing_radius_weight", 1.0);
  target_switch_score_margin_ = declare_parameter<double>("target_switch_score_margin", 0.5);
  target_switch_confirm_frames_ = std::max(1, static_cast<int>(
    declare_parameter<int>("target_switch_confirm_frames", 3)));
  target_switch_min_mean_y_advantage_ratio_ = declare_parameter<double>(
    "target_switch_min_mean_y_advantage_ratio", 0.08);
  target_switch_min_radius_ratio_ = declare_parameter<double>("target_switch_min_radius_ratio", 1.15);
  target_switch_max_vertical_mismatch_ratio_ = declare_parameter<double>(
    "target_switch_max_vertical_mismatch_ratio", 0.10);
  partial_gate_recovery_frames_ = std::max(1, static_cast<int>(
    declare_parameter<int>("partial_gate_recovery_frames", 12)));
  recovery_linear_speed_ = declare_parameter<double>("recovery_linear_speed", 0.15);
  recovery_yaw_rate_ = declare_parameter<double>("recovery_yaw_rate", 0.35);
  reassociation_max_position_ratio_ = declare_parameter<double>("reassociation_max_position_ratio", 0.12);
  reassociation_max_radius_ratio_ = declare_parameter<double>("reassociation_max_radius_ratio", 1.8);
  reassociation_max_vertical_mismatch_ratio_ = declare_parameter<double>(
    "reassociation_max_vertical_mismatch_ratio", 0.10);
  max_detection_age_s_ = declare_parameter<double>("max_detection_age_s", 1.0);
  passed_gate_suppression_time_s_ = declare_parameter<double>("passed_gate_suppression_time_s", 30.0);
  retired_position_ratio_ = declare_parameter<double>("retired_position_ratio", 0.12);
  retired_radius_ratio_ = declare_parameter<double>("retired_radius_ratio", 1.8);
  gate_3_clear_time_s_ = declare_parameter<double>("gate_3_clear_time_s", 3.0);
  gate_7_clear_time_s_ = declare_parameter<double>("gate_7_clear_time_s", 3.0);
  clear_forward_speed_ = declare_parameter<double>("clear_forward_speed", 0.3);
  clear_heading_kp_ = declare_parameter<double>("clear_heading_kp", 0.02);
  turn_angle_deg_ = declare_parameter<double>("turn_angle_deg", 90.0);
  turn_forward_speed_ = declare_parameter<double>("turn_forward_speed", 0.2);
  turn_angular_command_ = declare_parameter<double>("turn_angular_command", 0.4);
  turn_heading_tolerance_deg_ = declare_parameter<double>("turn_heading_tolerance_deg", 5.0);
  turn_stable_samples_ = std::max(1, static_cast<int>(declare_parameter<int>("turn_stable_samples", 5)));
  heading_timeout_s_ = declare_parameter<double>("heading_timeout_s", 1.0);
  detection_timeout_s_ = declare_parameter<double>("detection_timeout_s", 1.0);
  turn_timeout_s_ = declare_parameter<double>("turn_timeout_s", 30.0);
  for (double value : {score_size_reference_radius_, target_switch_min_radius_ratio_,
      recovery_linear_speed_, recovery_yaw_rate_, reassociation_max_radius_ratio_,
      passed_gate_suppression_time_s_, retired_position_ratio_,
      retired_radius_ratio_, gate_3_clear_time_s_, gate_7_clear_time_s_, clear_forward_speed_,
      clear_heading_kp_, turn_angle_deg_, turn_forward_speed_, turn_angular_command_,
      turn_heading_tolerance_deg_, heading_timeout_s_, detection_timeout_s_, turn_timeout_s_,
      max_detection_age_s_}) {
    if (!std::isfinite(value) || value <= 0) throw std::invalid_argument("gate parameters must be positive finite");
  }
  for (double weight : {score_center_weight_, score_vertical_weight_, score_size_weight_,
      score_confidence_weight_, score_continuity_weight_, score_vertical_mismatch_weight_,
      pairing_width_weight_, pairing_vertical_weight_, pairing_radius_weight_}) {
    if (!std::isfinite(weight) || weight < 0) throw std::invalid_argument("score weights must be finite/nonnegative");
  }
  for (double ratio : {target_switch_score_margin_, target_switch_min_mean_y_advantage_ratio_,
      target_switch_max_vertical_mismatch_ratio_, reassociation_max_position_ratio_,
      reassociation_max_vertical_mismatch_ratio_}) {
    if (!std::isfinite(ratio) || ratio < 0) throw std::invalid_argument("target recovery ratios must be finite/nonnegative");
  }
  if (turn_angle_deg_ >= 180 || turn_heading_tolerance_deg_ >= turn_angle_deg_) {
    throw std::invalid_argument("turn angle must exceed tolerance and be below 180 degrees");
  }
  if (steering_output_limit_ <= 0.0) {
    steering_output_limit_ = max_yaw_rate_;
  }
  steering_pid_.configure(
    heading_kp_, steering_ki_, steering_kd_, steering_output_limit_,
    steering_integral_limit_, steering_dt_min_, steering_dt_max_);

  if (use_object_tracking_) {
    tracked_sub_ = create_subscription<msgs::msg::TrackedObstacleArray>(
      "/asv/tracked_obstacles", rclcpp::QoS(1),
      std::bind(&BuoyCenteringNode::trackedCallback, this, std::placeholders::_1));
  } else {
    obstacle_sub_ = create_subscription<msgs::msg::ObstacleArray>(
      "/asv/obstacles", rclcpp::QoS(1),
      std::bind(&BuoyCenteringNode::obstacleCallback, this, std::placeholders::_1));
  }

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&BuoyCenteringNode::imageCallback, this, std::placeholders::_1));
  // The raw camera remains the fallback. The detector annotation is preferred
  // by the OSD so its YOLO boxes and labels remain visible in the web stream.
  detector_debug_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/asv/camera_debug", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&BuoyCenteringNode::detectorDebugImageCallback, this, std::placeholders::_1));
  mission_sub_ = this->create_subscription<msgs::msg::MissionStatus>(
    "/asv/mission_status", 10,
    std::bind(&BuoyCenteringNode::missionCallback, this, std::placeholders::_1));
  heading_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/compass_hdg", rclcpp::SensorDataQoS(),
    std::bind(&BuoyCenteringNode::headingCallback, this, std::placeholders::_1));
  mavros_state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&BuoyCenteringNode::mavrosStateCallback, this, std::placeholders::_1));

  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/asv/cmd_vel", 10);
  status_pub_ = this->create_publisher<msgs::msg::MissionStatus>("/asv/mission_status", 10);
  debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/asv/guidance_debug", 10);
  gate_count_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/asv/buoy_gate_count", 10);
  diagnostic_pub_ = this->create_publisher<msgs::msg::ControllerDiagnostic>(
    "/asv/control_diagnostic/buoy", 10);
  tracking_debug_pub_ = create_publisher<std_msgs::msg::String>("/asv/gate_debug", 1);
  control_timer_ = create_wall_timer(std::chrono::milliseconds(50),
    std::bind(&BuoyCenteringNode::controlTick, this));

  if (show_gui_) {
    try {
      cv::namedWindow("ASV Guidance & Buoy Centering Live Stream", cv::WINDOW_AUTOSIZE);
    } catch (const cv::Exception & e) {
      RCLCPP_WARN(this->get_logger(), "OpenCV GUI Window not available: %s", e.what());
      show_gui_ = false;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "buoy gate controller aktif: total_gates=%d search=%.2f m/s passing=%.2f m/s "
    "pass_radius_threshold=%.1f lost_frames_for_pass=%d min_pair_confidence=%.2f max_radius_ratio=%.2f",
    total_gates_, search_linear_speed_, passing_linear_speed_, pass_radius_threshold_,
    lost_frames_for_pass_, min_pair_confidence_, max_radius_ratio_);
  publishGateCount();
}

BuoyCenteringNode::~BuoyCenteringNode()
{
  if (show_gui_) {
    cv::destroyAllWindows();
  }
}

void BuoyCenteringNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    latest_frame_ = cv_ptr->image.clone();
    latest_header_ = msg->header;
    has_frame_ = true;

    if (msg->width > 0) {
      frame_width_ = static_cast<double>(msg->width);
    }
    if (msg->height > 0) {
      frame_height_ = static_cast<double>(msg->height);
    }
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void BuoyCenteringNode::missionCallback(const msgs::msg::MissionStatus::SharedPtr msg)
{
  mission_state_ = msg->state;
}

void BuoyCenteringNode::detectorDebugImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    latest_detector_debug_frame_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    latest_detector_debug_header_ = msg->header;
    has_detector_debug_frame_ = true;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN(this->get_logger(), "debug image cv_bridge exception: %s", e.what());
  }
}

void BuoyCenteringNode::headingCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  heading_deg_ = msg->data;
  has_heading_ = std::isfinite(heading_deg_);
  if (has_heading_) heading_received_ = now().seconds();
  if (state_ == ControllerState::TURNING || state_ == ControllerState::TURN_STABILIZING) {
    if (has_heading_ && std::abs(gate_logic::wrap(turn_target_heading_ - heading_deg_)) <=
      turn_heading_tolerance_deg_) {
      ++stable_samples_;
      if (state_ == ControllerState::TURNING) transitionTo(ControllerState::TURN_STABILIZING);
    } else {
      stable_samples_ = 0;
      transitionTo(ControllerState::TURNING);
    }
  }
}

void BuoyCenteringNode::mavrosStateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  mavros_state_ = *msg;
  has_mavros_state_ = true;
}

void BuoyCenteringNode::obstacleCallback(const msgs::msg::ObstacleArray::SharedPtr msg)
{
  obstacle_received_ = now().seconds();
  visible_locked_ = 0;
  partial_recovery_ = false;
  recovery_missing_ = "NONE";
  reassociation_debug_ = "none";
  if (debug_pairing_ && state_ != ControllerState::SEARCHING) {
    (void)extractPair(*msg);  // debug only: never controls an existing lock/manoeuvre
  }
  PairDetection pair;
  bool target_detected = false;
  double linear_x = 0.0;
  double angular_z = 0.0;
  double midpoint_x = -1.0;
  double error_x = 0.0;

  switch (state_) {
    case ControllerState::SEARCHING: {
      updateRecentlyPassedTarget(*msg);
      pair = extractPair(
        *msg, has_recently_passed_target_ ? &recently_passed_target_ : nullptr);
      if (pair.valid) {
        candidate_frames_ = matchesTarget(pair, pending_candidate_) ? candidate_frames_ + 1 : 1;
        pending_candidate_ = pair;
        if (candidate_frames_ < candidate_confirm_frames_) pair.valid = false;
      } else {
        candidate_frames_ = 0;
        pending_candidate_ = PairDetection{};
      }
      if (pair.valid) {
        resetSwitchCandidate("new_target_locked");
        current_target_ = pair;
        has_current_target_ = true;
        approaching_target_lost_frames_ = 0;
        transitionTo(ControllerState::APPROACHING);
        handleApproaching(current_target_);
        target_detected = true;
      } else {
        handleSearching();
      }
      break;
    }

    case ControllerState::APPROACHING: {
      pair = has_current_target_ ? findAssociatedPair(*msg, current_target_) : PairDetection{};
      BuoyObservation red;
      BuoyObservation green;
      const bool red_visible = has_current_target_ && findObservation(
        *msg, msgs::msg::ObstacleArray::RED_BUOY, current_target_.red_id, red);
      const bool green_visible = has_current_target_ && findObservation(
        *msg, msgs::msg::ObstacleArray::GREEN_BUOY, current_target_.green_id, green);
      visible_locked_ = static_cast<int>(red_visible) + static_cast<int>(green_visible);
      if (!pair.valid && visible_locked_ == 1) {
        PairDetection recovered;
        if (tryReassociate(*msg, current_target_, red, green, recovered)) {
          pair = recovered;
          visible_locked_ = 2;
        }
      }
      if (!pair.valid) {
        resetSwitchCandidate("locked_gate_incomplete");
        ++approaching_target_lost_frames_;
        const int recovery_limit = visible_locked_ == 1 ?
          partial_gate_recovery_frames_ : approaching_lost_frames_tolerance_;
        if (approaching_target_lost_frames_ >= recovery_limit) {
          clearCurrentTarget();
          near_gate_ = false;
          transitionTo(ControllerState::SEARCHING);
          publishCmd(0.0, 0.0);
        } else if (visible_locked_ == 1) {
          partial_recovery_ = true;
          const bool missing_red = !red_visible;
          recovery_missing_ = missing_red ? "RED" : "GREEN";
          const BuoyObservation & visible = red_visible ? red : green;
          const PairDetection recovery_target = current_target_;
          if (red_visible) {
            current_target_.red_x = red.x;
            current_target_.red_y = red.y;
            current_target_.red_radius = red.radius;
            current_target_.red_confidence = red.confidence;
          } else {
            current_target_.green_x = green.x;
            current_target_.green_y = green.y;
            current_target_.green_radius = green.radius;
            current_target_.green_confidence = green.confidence;
          }
          pair = current_target_;
          handlePartialRecovery(visible, missing_red, recovery_target);
        } else if (has_current_target_) {
          pair = current_target_;
          publishCmd(0.0, std::clamp(last_angular_z_, -recovery_yaw_rate_, recovery_yaw_rate_));
        }
      } else {
        current_target_ = pair;
        approaching_target_lost_frames_ = 0;
        target_detected = true;
        PairDetection switched;
        if (considerTargetSwitch(*msg, current_target_, switched)) {
          RCLCPP_INFO(get_logger(), "Switch target %ld/%ld -> %ld/%ld after %d-frame confirmation",
            current_target_.red_id, current_target_.green_id, switched.red_id, switched.green_id,
            target_switch_confirm_frames_);
          current_target_ = switched;
          pair = switched;
        }
        if (isNearGate(current_target_)) {
          near_gate_ = true;
          lost_frames_ = 0;
          passing_pair_ = current_target_;
          passing_entry_min_radius_ = std::min(
            passing_pair_.red_radius, passing_pair_.green_radius);
          transitionTo(ControllerState::PASSING);
          handlePassing();
        } else {
          handleApproaching(current_target_);
        }
      }
      break;
    }

    case ControllerState::PASSING: {
      pair = findAssociatedPair(*msg, passing_pair_);
      BuoyObservation red;
      BuoyObservation green;
      const bool red_visible = findObservation(
        *msg, msgs::msg::ObstacleArray::RED_BUOY, passing_pair_.red_id, red);
      const bool green_visible = findObservation(
        *msg, msgs::msg::ObstacleArray::GREEN_BUOY, passing_pair_.green_id, green);
      visible_locked_ = static_cast<int>(red_visible) + static_cast<int>(green_visible);
      if (!pair.valid && visible_locked_ == 1) {
        PairDetection recovered;
        if (tryReassociate(*msg, passing_pair_, red, green, recovered)) {
          pair = recovered;
          visible_locked_ = 2;
        }
      }
      if (pair.valid) {
        passing_pair_ = pair;
        current_target_ = pair;
      }
      if (visible_locked_ == 0) ++lost_frames_;
      else lost_frames_ = 0;

      handlePassing();
      if (gate_logic::lossAction(true, near_gate_, visible_locked_, lost_frames_,
          lost_frames_for_pass_) == gate_logic::LossAction::MARK_PASSED) {
        markGatePassed();
        if (state_ == ControllerState::PASSING) {
          transitionTo(ControllerState::SEARCHING);
          handleSearching();
        }
      }
      break;
    }

    case ControllerState::CLEARING_GATE:
    case ControllerState::TURNING:
    case ControllerState::TURN_STABILIZING:
      // Perception continues, but only the independent heading controller commands motion.
      break;

    case ControllerState::FINISHED:
      // Latched stop: later detections must never restart the controller.
      publishCmd(0.0, 0.0);
      break;
  }

  if (pair.valid) {
    midpoint_x = (pair.red_x + pair.green_x) / 2.0;
    const double center_x = frame_width_ / 2.0;
    error_x = (midpoint_x - center_x) / center_x;
  }

  if (state_ == ControllerState::APPROACHING) {
    const double speed_factor = std::max(0.2, 1.0 - std::abs(error_x) * speed_reduction_factor_);
    linear_x = max_linear_speed_ * speed_factor;
    angular_z = last_angular_z_;
  } else if (state_ == ControllerState::SEARCHING) {
    const auto search_command = searchingCommand();
    linear_x = search_command.first;
    angular_z = search_command.second;
    last_p_ = 0.0;
    last_i_ = 0.0;
    last_d_ = 0.0;
    last_angular_z_ = 0.0;
  } else if (state_ == ControllerState::PASSING) {
    linear_x = passing_linear_speed_;
    last_p_ = 0.0;
    last_i_ = 0.0;
    last_d_ = 0.0;
    last_angular_z_ = 0.0;
  }

  linear_x = commanded_linear_;
  angular_z = commanded_angular_;
  publishTrackingDebug(msg->positions.size());
  publishDiagnostic(target_detected && state_ == ControllerState::APPROACHING,
    error_x, linear_x, angular_z, last_p_, last_i_, last_d_);

  renderOverlay(latest_frame_, msg, pair, midpoint_x, error_x, linear_x, angular_z, state_);
}

std::vector<BuoyCenteringNode::PairDetection> BuoyCenteringNode::extractValidPairs(
  const msgs::msg::ObstacleArray & msg) const
{
  std::vector<PairDetection> candidates;
  const size_t count = std::min({msg.positions.size(), msg.radii.size(), msg.classes.size()});
  for (size_t red_index = 0; red_index < count; ++red_index) {
    if (msg.classes[red_index] != msgs::msg::ObstacleArray::RED_BUOY) continue;
    for (size_t green_index = 0; green_index < count; ++green_index) {
      if (msg.classes[green_index] != msgs::msg::ObstacleArray::GREEN_BUOY) continue;
      PairDetection candidate;
      candidate.session = tracker_session_;
      if (red_index < frame_ids_.size()) candidate.red_id = frame_ids_[red_index];
      if (green_index < frame_ids_.size()) candidate.green_id = frame_ids_[green_index];
      candidate.red_index = red_index;
      candidate.green_index = green_index;
      candidate.red_x = msg.positions[red_index].x;
      candidate.red_y = msg.positions[red_index].y;
      candidate.green_x = msg.positions[green_index].x;
      candidate.green_y = msg.positions[green_index].y;
      candidate.red_radius = msg.radii[red_index];
      candidate.green_radius = msg.radii[green_index];
      candidate.red_confidence = red_index < msg.confidences.size() ? msg.confidences[red_index] : 0.0F;
      candidate.green_confidence = green_index < msg.confidences.size() ? msg.confidences[green_index] : 0.0F;
      const float smaller_radius = std::min(candidate.red_radius, candidate.green_radius);
      const float larger_radius = std::max(candidate.red_radius, candidate.green_radius);
      const double radius_ratio = smaller_radius > 0.0F ? larger_radius / smaller_radius : 0.0;
      const double horizontal_distance = std::abs(candidate.red_x - candidate.green_x);
      const double vertical_distance = std::abs(candidate.red_y - candidate.green_y);
      const bool geometric_pair =
        horizontal_distance >= frame_width_ * min_pair_horizontal_separation_ratio_ &&
        horizontal_distance <= frame_width_ * max_pair_horizontal_separation_ratio_ &&
        vertical_distance <= frame_height_ * max_pair_vertical_offset_ratio_;
      candidate.valid = smaller_radius > 0.0F &&
        candidate.red_confidence >= min_pair_confidence_ &&
        candidate.green_confidence >= min_pair_confidence_ &&
        radius_ratio <= max_radius_ratio_ && geometric_pair;
      candidate.valid = candidate.valid && std::isfinite(candidate.red_x) &&
        std::isfinite(candidate.green_x) && std::isfinite(candidate.red_y) &&
        std::isfinite(candidate.green_y) && std::isfinite(larger_radius) &&
        (!use_object_tracking_ || (candidate.red_id >= 0 && candidate.green_id >= 0 &&
        candidate.red_id != candidate.green_id && !candidate.session.empty())) &&
        (expected_gate_color_order_ == "any" ||
        (expected_gate_color_order_ == "red_left" && candidate.red_x < candidate.green_x) ||
        (expected_gate_color_order_ == "green_left" && candidate.green_x < candidate.red_x));
      if (!candidate.valid && debug_pairing_) {
        RCLCPP_DEBUG(get_logger(), "reject %ld/%ld: confidence=%.2f/%.2f radius_ratio=%.2f "
          "dx=%.1f dy=%.1f order=%s tracked=%d",
          candidate.red_id, candidate.green_id, candidate.red_confidence, candidate.green_confidence,
          radius_ratio, horizontal_distance, vertical_distance, expected_gate_color_order_.c_str(),
          use_object_tracking_);
      }
      if (candidate.valid) {
        candidates.push_back(candidate);
      }
    }
  }
  return candidates;
}

std::vector<BuoyCenteringNode::PairDetection> BuoyCenteringNode::extractAssignedPairs(
  const msgs::msg::ObstacleArray & msg) const
{
  const auto candidates = extractValidPairs(msg);
  std::vector<gate_logic::Edge> edges;
  for (size_t i = 0; i < candidates.size(); ++i) {
    const auto & candidate = candidates[i];
    if (retired(candidate)) continue;
    const double cost = gate_logic::pairAffinityCost(
      candidate.red_x, candidate.red_y, candidate.red_radius,
      candidate.green_x, candidate.green_y, candidate.green_radius,
      frame_width_, frame_height_, pairing_width_weight_, pairing_vertical_weight_,
      pairing_radius_weight_);
    edges.push_back({candidate.red_index, candidate.green_index, cost, i});
  }
  std::vector<PairDetection> assigned;
  for (const size_t index : gate_logic::assign(edges)) {
    assigned.push_back(candidates[index]);
  }
  return assigned;
}

bool BuoyCenteringNode::matchesTarget(
  const PairDetection & candidate, const PairDetection & target) const
{
  if (!candidate.valid || !target.valid ||
    candidate.red_radius <= 0.0F || candidate.green_radius <= 0.0F ||
    target.red_radius <= 0.0F || target.green_radius <= 0.0F)
  {
    return false;
  }
  const double max_position_distance = frame_width_ * target_association_max_position_ratio_;
  // Never substitute a new track or session into an existing tracked lock.
  if (target.red_id >= 0 || target.green_id >= 0) {
    if (candidate.session != target.session || candidate.red_id != target.red_id ||
      candidate.green_id != target.green_id) return false;
  }
  const auto distance = [](double x1, double y1, double x2, double y2) {
      return std::hypot(x1 - x2, y1 - y2);
    };
  const auto radius_ratio = [](float current, float previous) {
      return std::max(current, previous) / std::min(current, previous);
    };
  return distance(candidate.red_x, candidate.red_y, target.red_x, target.red_y) <=
           max_position_distance &&
         distance(candidate.green_x, candidate.green_y, target.green_x, target.green_y) <=
           max_position_distance &&
         radius_ratio(candidate.red_radius, target.red_radius) <= target_association_max_radius_ratio_ &&
         radius_ratio(candidate.green_radius, target.green_radius) <= target_association_max_radius_ratio_;
}

BuoyCenteringNode::PairDetection BuoyCenteringNode::findAssociatedPair(
  const msgs::msg::ObstacleArray & msg, const PairDetection & target) const
{
  PairDetection associated;
  double best_score = std::numeric_limits<double>::infinity();
  for (const auto & candidate : extractValidPairs(msg)) {
    if (!matchesTarget(candidate, target)) continue;
    const double position_score =
      std::hypot(candidate.red_x - target.red_x, candidate.red_y - target.red_y) +
      std::hypot(candidate.green_x - target.green_x, candidate.green_y - target.green_y);
    const double radius_score =
      std::abs(candidate.red_radius - target.red_radius) +
      std::abs(candidate.green_radius - target.green_radius);
    const double score = position_score + 0.25 * radius_score;
    if (score < best_score) {
      associated = candidate;
      best_score = score;
    }
  }
  return associated;
}

bool BuoyCenteringNode::findObservation(
  const msgs::msg::ObstacleArray & msg, uint8_t buoy_class, int64_t id,
  BuoyObservation & observation) const
{
  observation = BuoyObservation{};
  if (id < 0) return false;
  const size_t count = std::min({msg.positions.size(), msg.radii.size(), msg.classes.size(),
      msg.confidences.size(), frame_ids_.size()});
  for (size_t i = 0; i < count; ++i) {
    if (frame_ids_[i] != id || msg.classes[i] != buoy_class || msg.radii[i] <= 0.0F ||
      !std::isfinite(msg.positions[i].x) || !std::isfinite(msg.positions[i].y)) continue;
    observation.valid = true;
    observation.id = id;
    observation.index = i;
    observation.x = msg.positions[i].x;
    observation.y = msg.positions[i].y;
    observation.radius = msg.radii[i];
    observation.confidence = msg.confidences[i];
    return true;
  }
  return false;
}

bool BuoyCenteringNode::tryReassociate(
  const msgs::msg::ObstacleArray & msg, const PairDetection & target,
  const BuoyObservation & red, const BuoyObservation & green,
  PairDetection & recovered)
{
  recovered = PairDetection{};
  if (red.valid == green.valid || target.session != tracker_session_) return false;
  double best_cost = std::numeric_limits<double>::infinity();
  for (const auto & candidate : extractAssignedPairs(msg)) {
    const bool replace_green = red.valid && candidate.red_id == target.red_id &&
      candidate.green_id != target.green_id;
    const bool replace_red = green.valid && candidate.green_id == target.green_id &&
      candidate.red_id != target.red_id;
    if (!replace_green && !replace_red) continue;

    const double old_x = replace_green ? target.green_x : target.red_x;
    const double old_y = replace_green ? target.green_y : target.red_y;
    const double new_x = replace_green ? candidate.green_x : candidate.red_x;
    const double new_y = replace_green ? candidate.green_y : candidate.red_y;
    const double old_radius = replace_green ? target.green_radius : target.red_radius;
    const double new_radius = replace_green ? candidate.green_radius : candidate.red_radius;
    const double position_distance = std::hypot(new_x - old_x, new_y - old_y);
    const double radius_ratio = std::max(old_radius, new_radius) /
      std::min(old_radius, new_radius);
    const double mismatch = gate_logic::verticalMismatch(
      candidate.red_y, candidate.green_y, frame_height_);
    if (!gate_logic::reassociationCompatible(
        position_distance, frame_width_ * reassociation_max_position_ratio_,
        radius_ratio, reassociation_max_radius_ratio_, mismatch,
        reassociation_max_vertical_mismatch_ratio_)) continue;
    const double cost = position_distance / frame_width_ +
      std::abs(std::log(radius_ratio)) + mismatch;
    if (cost < best_cost) {
      recovered = candidate;
      best_cost = cost;
    }
  }
  if (!recovered.valid) return false;
  const int64_t old_id = red.valid ? target.green_id : target.red_id;
  const int64_t new_id = red.valid ? recovered.green_id : recovered.red_id;
  reassociation_debug_ = std::to_string(old_id) + "->" + std::to_string(new_id);
  return true;
}

double BuoyCenteringNode::targetScore(const PairDetection & pair) const
{
  const double base = gate_logic::selectionScore(
    (pair.red_x + pair.green_x) / 2, (pair.red_y + pair.green_y) / 2,
    (pair.red_radius + pair.green_radius) / 2,
    (pair.red_confidence + pair.green_confidence) / 2, frame_width_, frame_height_,
    score_size_reference_radius_, score_center_weight_, score_vertical_weight_,
    score_size_weight_, score_confidence_weight_);
  return gate_logic::rationalTargetScore(
    base, pair.red_y, pair.green_y, frame_height_, score_vertical_mismatch_weight_);
}

void BuoyCenteringNode::resetSwitchCandidate(const std::string & reason)
{
  switch_candidate_ = PairDetection{};
  switch_confirm_frames_ = 0;
  switch_reason_ = reason;
}

bool BuoyCenteringNode::considerTargetSwitch(
  const msgs::msg::ObstacleArray & msg, const PairDetection & current,
  PairDetection & selected)
{
  selected = PairDetection{};
  const PairDetection candidate = extractPair(msg, &current);
  if (!candidate.valid) {
    resetSwitchCandidate("no_candidate");
    return false;
  }
  const double current_mean_y = (current.red_y + current.green_y) / 2;
  const double candidate_mean_y = (candidate.red_y + candidate.green_y) / 2;
  const double current_mean_radius = (current.red_radius + current.green_radius) / 2;
  const double candidate_mean_radius = (candidate.red_radius + candidate.green_radius) / 2;
  const bool qualifies = gate_logic::shouldSwitchTarget(
    targetScore(current), targetScore(candidate), current_mean_y, candidate_mean_y,
    current_mean_radius, candidate_mean_radius,
    gate_logic::verticalMismatch(candidate.red_y, candidate.green_y, frame_height_),
    target_switch_score_margin_, frame_height_ * target_switch_min_mean_y_advantage_ratio_,
    target_switch_min_radius_ratio_, target_switch_max_vertical_mismatch_ratio_);
  if (!qualifies) {
    resetSwitchCandidate("insufficient_improvement");
    return false;
  }
  const bool same_candidate = switch_candidate_.valid &&
    switch_candidate_.session == candidate.session &&
    switch_candidate_.red_id == candidate.red_id && switch_candidate_.green_id == candidate.green_id;
  switch_confirm_frames_ = same_candidate ? switch_confirm_frames_ + 1 : 1;
  switch_candidate_ = candidate;
  switch_reason_ = "confirming_rational_near_gate";
  if (switch_confirm_frames_ < target_switch_confirm_frames_) return false;
  selected = candidate;
  selected_score_ = targetScore(candidate);
  switch_reason_ = "switched_rational_near_gate";
  switch_confirm_frames_ = 0;
  return true;
}

BuoyCenteringNode::PairDetection BuoyCenteringNode::extractPair(
  const msgs::msg::ObstacleArray & msg, const PairDetection * excluded_target) const
{
  const auto candidates = extractAssignedPairs(msg);
  debug_candidates_.clear();
  PairDetection pair;
  double best_score = -std::numeric_limits<double>::infinity();
  for (const auto & candidate : candidates) {
    debug_candidates_.push_back(candidate);
    if ((excluded_target != nullptr && matchesTarget(candidate, *excluded_target)))
    {
      continue;
    }
    const double score = targetScore(candidate) +
      (matchesTarget(candidate, pending_candidate_) ? score_continuity_weight_ : 0.0);
    if (score > best_score) {
      pair = candidate;
      best_score = score;
      selected_score_ = score;
    }
  }
  return pair;
}

void BuoyCenteringNode::clearCurrentTarget()
{
  current_target_ = PairDetection{};
  candidate_frames_ = 0;
  pending_candidate_ = PairDetection{};
  passing_pair_ = PairDetection{};
  has_current_target_ = false;
  approaching_target_lost_frames_ = 0;
  lost_frames_ = 0;
  passing_entry_min_radius_ = 0.0F;
  resetSwitchCandidate("target_cleared");
  visible_locked_ = 0;
  partial_recovery_ = false;
  recovery_missing_ = "NONE";
}

void BuoyCenteringNode::updateRecentlyPassedTarget(const msgs::msg::ObstacleArray & msg)
{
  (void)msg;
  if (!has_recently_passed_target_) return;
  if (--recently_passed_target_block_frames_ <= 0) {
    recently_passed_target_ = PairDetection{};
    has_recently_passed_target_ = false;
    recently_passed_target_block_frames_ = 0;
  }
}

void BuoyCenteringNode::handleSearching()
{
  const auto command = searchingCommand();
  publishCmd(command.first, command.second);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "SEARCHING | gate %d/%d | maju pelan mencari pasangan buoy",
    gate_count_, total_gates_);
}

std::pair<double, double> BuoyCenteringNode::searchingCommand() const
{
  // gate_count_ is the number of completed pairs, incremented only by
  // markGatePassed(). It therefore directly represents passed_pair_count.
  if (gate_count_ == 0) {
    return {0.3, 0.0};
  }
  return {search_linear_speed_, 0.0};
}

void BuoyCenteringNode::handleApproaching(const PairDetection & pair)
{
  const double midpoint_x = (pair.red_x + pair.green_x) / 2.0;
  const double center_x = frame_width_ / 2.0;
  const double error_x = (midpoint_x - center_x) / center_x;
  const auto now = this->get_clock()->now();
  const double dt = last_control_time_.nanoseconds() == 0 ? 0.0 :
    (now - last_control_time_).seconds();
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double angular_z = steering_pid_.update(error_x, dt, p, i, d);
  angular_z = std::clamp(angular_z, -max_yaw_rate_, max_yaw_rate_);
  last_control_time_ = now;
  last_p_ = p;
  last_i_ = i;
  last_d_ = d;
  last_angular_z_ = angular_z;
  const double speed_factor = std::max(0.2, 1.0 - std::abs(error_x) * speed_reduction_factor_);
  const double linear_x = max_linear_speed_ * speed_factor;

  publishCmd(linear_x, angular_z);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 500,
    "APPROACHING | gate %d/%d | Red_X: %.1f Green_X: %.1f Midpoint: %.1f Error: %.2f | Cmd V: %.2f W: %.2f",
    gate_count_, total_gates_, pair.red_x, pair.green_x, midpoint_x, error_x, linear_x, angular_z);
}

void BuoyCenteringNode::handlePartialRecovery(
  const BuoyObservation & visible, bool missing_red, const PairDetection & target)
{
  const double previous_visible_x = missing_red ? target.green_x : target.red_x;
  const double previous_missing_x = missing_red ? target.red_x : target.green_x;
  const double recovery_aim_x = gate_logic::recoveryAim(
    visible.x, previous_visible_x, previous_missing_x);
  const double center_x = frame_width_ / 2.0;
  const double error_x = (recovery_aim_x - center_x) / center_x;
  const auto now = this->get_clock()->now();
  const double dt = last_control_time_.nanoseconds() == 0 ? 0.0 :
    (now - last_control_time_).seconds();
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double angular_z = steering_pid_.update(error_x, dt, p, i, d);
  angular_z = std::clamp(angular_z, -recovery_yaw_rate_, recovery_yaw_rate_);
  last_control_time_ = now;
  last_p_ = p;
  last_i_ = i;
  last_d_ = d;
  last_angular_z_ = angular_z;
  publishCmd(recovery_linear_speed_, angular_z);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
    "PARTIAL_RECOVERY | missing=%s aim=%.1f error=%.2f | Cmd V: %.2f W: %.2f",
    missing_red ? "RED" : "GREEN", recovery_aim_x, error_x, recovery_linear_speed_, angular_z);
}

void BuoyCenteringNode::handlePassing()
{
  publishCmd(passing_linear_speed_, 0.0);
  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 1000,
    "PASSING | gate %d/%d | lost_frames=%d/%d | maju melewati gate",
    gate_count_ + 1, total_gates_, lost_frames_, lost_frames_for_pass_);
}

void BuoyCenteringNode::markGatePassed()
{
  ++gate_count_;
  publishGateCount();
  RCLCPP_INFO(
    this->get_logger(), "Gate %d/%d passed", gate_count_, total_gates_);
  recently_passed_target_ = passing_pair_;
  retired_gates_.push_back({passing_pair_, now().seconds()});
  has_recently_passed_target_ = recently_passed_target_.valid;
  recently_passed_target_block_frames_ = passed_gate_reacquire_block_frames_;
  near_gate_ = false;
  clearCurrentTarget();

  if (gate_count_ >= total_gates_) {
    transitionTo(ControllerState::FINISHED);
    publishCmd(0.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Gate %d/%d passed - FINISHED - STOP", gate_count_, total_gates_);
  } else if (gate_logic::specialGate(gate_count_)) {
    transitionTo(ControllerState::CLEARING_GATE);
    clearing_elapsed_ = 0;
    tick_time_ = now().seconds();
    clear_heading_captured_ = false;
    stable_samples_ = 0;
    publishCmd(0, 0);
  }
}

void BuoyCenteringNode::transitionTo(ControllerState next_state)
{
  if (state_ == next_state) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "%s -> %s", stateName(), stateName(next_state));
  state_ = next_state;
  if (maneuverActive()) debug_candidates_.clear();
  steering_pid_.reset();
  last_control_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  last_p_ = 0.0;
  last_i_ = 0.0;
  last_d_ = 0.0;
  last_angular_z_ = 0.0;
}

const char * BuoyCenteringNode::stateName() const
{
  return stateName(state_);
}

const char * BuoyCenteringNode::stateName(ControllerState state) const
{
  switch (state) {
    case ControllerState::SEARCHING: return "SEARCHING";
    case ControllerState::APPROACHING: return "APPROACHING";
    case ControllerState::PASSING: return "PASSING";
    case ControllerState::CLEARING_GATE: return "CLEARING_GATE";
    case ControllerState::TURNING: return "TURNING";
    case ControllerState::TURN_STABILIZING: return "TURN_STABILIZING";
    case ControllerState::FINISHED: return "FINISHED";
  }
  return "UNKNOWN";
}

const char * BuoyCenteringNode::missionStateName(uint8_t state) const
{
  switch (state) {
    case msgs::msg::MissionStatus::IDLE: return "IDLE";
    case msgs::msg::MissionStatus::PREPARATION: return "PREPARATION";
    case msgs::msg::MissionStatus::START: return "START";
    case msgs::msg::MissionStatus::NAVIGATING_BUOYS: return "NAVIGATING_BUOYS";
    case msgs::msg::MissionStatus::SURFACE_IMAGING: return "SURFACE_IMAGING";
    case msgs::msg::MissionStatus::UNDERWATER_IMAGING: return "UNDERWATER_IMAGING";
    case msgs::msg::MissionStatus::DOCKING: return "DOCKING";
    case msgs::msg::MissionStatus::FINISH: return "FINISH";
    case msgs::msg::MissionStatus::PAUSED: return "PAUSED";
    case msgs::msg::MissionStatus::COMPLETED: return "COMPLETED";
    case msgs::msg::MissionStatus::ABORTED: return "ABORTED";
    case msgs::msg::MissionStatus::TIMEOUT: return "TIMEOUT";
    default: return "UNKNOWN";
  }
}

bool BuoyCenteringNode::isNearGate(const PairDetection & pair) const
{
  return pair.red_radius >= pass_radius_threshold_ &&
    pair.green_radius >= pass_radius_threshold_;
}

void BuoyCenteringNode::publishGateCount()
{
  std_msgs::msg::UInt8 msg;
  msg.data = static_cast<uint8_t>(std::min(gate_count_, 255));
  gate_count_pub_->publish(msg);
}

void BuoyCenteringNode::renderOverlay(
  cv::Mat & frame,
  const msgs::msg::ObstacleArray::SharedPtr & obs_msg,
  const PairDetection & pair,
  double midpoint_x, double error_x,
  double cmd_linear, double cmd_angular,
  ControllerState state)
{
  cv::Mat display_frame;
  if (has_detector_debug_frame_ && !latest_detector_debug_frame_.empty()) {
    display_frame = latest_detector_debug_frame_.clone();
  } else if (has_frame_ && !frame.empty()) {
    display_frame = frame.clone();
  } else {
    display_frame = cv::Mat(static_cast<int>(frame_height_), static_cast<int>(frame_width_), CV_8UC3, cv::Scalar(20, 20, 20));
    cv::putText(display_frame, "NO CAMERA STREAM RECEIVED", cv::Point(150, 240),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
  }

  int w = display_frame.cols;
  int h = display_frame.rows;
  int center_x = w / 2;

  // 1. Draw frame vertical centerline
  cv::line(display_frame, cv::Point(center_x, 40), cv::Point(center_x, h - 30), cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  // 2. Draw all detected balls/buoys from obstacle array
  const size_t count = std::min({obs_msg->positions.size(), obs_msg->radii.size(), obs_msg->classes.size()});
  for (size_t i = 0; i < count; ++i) {
    cv::Point2f pt(obs_msg->positions[i].x, obs_msg->positions[i].y);
    float radius = obs_msg->radii[i];
    uint8_t cls = obs_msg->classes[i];

    cv::Scalar circle_color(255, 255, 255);
    std::string tag = "OBJECT";

    if (cls == msgs::msg::ObstacleArray::RED_BUOY) {
      circle_color = cv::Scalar(0, 0, 255);
      tag = "RED BUOY";
    } else if (cls == msgs::msg::ObstacleArray::GREEN_BUOY) {
      circle_color = cv::Scalar(0, 255, 0);
      tag = "GREEN BUOY";
    } else if (cls == msgs::msg::ObstacleArray::BLUE_DOCKING_BUOY) {
      circle_color = cv::Scalar(255, 0, 0);
      tag = "BLUE DOCK";
    }

    cv::circle(display_frame, pt, static_cast<int>(radius), circle_color, 2, cv::LINE_AA);
    cv::circle(display_frame, pt, 4, circle_color, -1);
    cv::putText(display_frame, tag, cv::Point(pt.x - 25, pt.y - radius - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, circle_color, 2);
  }

  // 3. Draw Centering vector lines & Midpoint if active
  for (const auto & candidate : debug_candidates_) {
    cv::line(display_frame, cv::Point(candidate.red_x, candidate.red_y),
      cv::Point(candidate.green_x, candidate.green_y), cv::Scalar(140, 140, 140), 1);
  }
  if (pair.valid && state != ControllerState::FINISHED) {
    cv::Point red_pt(static_cast<int>(pair.red_x), static_cast<int>(pair.red_y));
    cv::Point green_pt(static_cast<int>(pair.green_x), static_cast<int>(pair.green_y));
    cv::Point mid_pt(
      static_cast<int>(midpoint_x), static_cast<int>((pair.red_y + pair.green_y) / 2.0));

    // Line connecting Red and Green Buoys
    cv::line(display_frame, red_pt, green_pt, cv::Scalar(255, 255, 0), 2, cv::LINE_AA);

    // Midpoint marker
    cv::drawMarker(display_frame, mid_pt, cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 20, 2);

    // Line from frame center to midpoint
    cv::line(display_frame, cv::Point(center_x, mid_pt.y), mid_pt, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);

    // Steering Indicator Bar at bottom
    int bar_y = h - 20;
    int error_bar_len = static_cast<int>(error_x * (center_x - 40));
    cv::line(display_frame, cv::Point(center_x, bar_y), cv::Point(center_x + error_bar_len, bar_y), cv::Scalar(0, 255, 255), 4);
    cv::circle(display_frame, cv::Point(center_x + error_bar_len, bar_y), 6, cv::Scalar(0, 255, 255), -1);
  }

  // 4. Render Top OSD Status Banner
  cv::Rect banner_rect(0, 0, w, 64);
  cv::Scalar banner_bg = state == ControllerState::FINISHED ? cv::Scalar(0, 0, 180) : cv::Scalar(0, 160, 0);
  cv::rectangle(display_frame, banner_rect, banner_bg, -1);

  std::string status_str = cv::format("[ CTRL: %s | GATE %d/%d ]", stateName(), gate_count_, total_gates_);
  cv::putText(display_frame, status_str, cv::Point(10, 24),
              cv::FONT_HERSHEY_SIMPLEX, 0.65, cv::Scalar(255, 255, 255), 2);
  const std::string heading = has_heading_ ? cv::format("%.1f deg", heading_deg_) : "N/A";
  const std::string fcu = has_mavros_state_ ?
    cv::format("%s %s %s", mavros_state_.mode.c_str(),
      mavros_state_.armed ? "ARMED" : "DISARMED",
      mavros_state_.connected ? "FCU OK" : "FCU LOST") : "FCU N/A";
  const std::string mission_str = cv::format("MISSION: %s | HDG: %s | %s",
    missionStateName(mission_state_), heading.c_str(), fcu.c_str());
  cv::putText(display_frame, mission_str, cv::Point(10, 53),
              cv::FONT_HERSHEY_SIMPLEX, 0.48, cv::Scalar(255, 255, 255), 1);

  // 5. Render Command HUD Text at bottom left
  std::string cmd_str = cv::format("Cmd Vel -> Linear: %.2f m/s | Yaw: %.2f rad/s", cmd_linear, cmd_angular);
  cv::putText(display_frame, cmd_str, cv::Point(10, h - 35),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

  // 6. Publish annotated debug image for ROS topic / GCS
  std_msgs::msg::Header header;
  if (has_detector_debug_frame_) {
    header = latest_detector_debug_header_;
  } else if (has_frame_) {
    header = latest_header_;
  } else {
    header.stamp = this->get_clock()->now();
  }
  cv_bridge::CvImage debug_msg(header, "bgr8", display_frame);
  debug_img_pub_->publish(*debug_msg.toImageMsg());

  // 7. Show live OpenCV GUI window if show_gui parameter is true
  if (show_gui_) {
    cv::imshow("ASV Guidance & Buoy Centering Live Stream", display_frame);
    cv::waitKey(1);
  }
}

void BuoyCenteringNode::publishCmd(double linear_x, double angular_z)
{
  commanded_linear_ = linear_x;
  commanded_angular_ = angular_z;
  geometry_msgs::msg::Twist msg;
  msg.linear.x = linear_x;
  msg.angular.z = angular_z;
  cmd_pub_->publish(msg);
}

void BuoyCenteringNode::trackedCallback(const msgs::msg::TrackedObstacleArray::SharedPtr msg)
{
  const auto & o = msg->obstacles;
  std::set<int64_t> unique_ids;
  for (auto id : msg->track_ids) {
    if (id >= 0 && !unique_ids.insert(id).second) {
      publishCmd(0, 0);
      return;
    }
  }
  const double stamp = rclcpp::Time(o.header.stamp).seconds();
  if (stamp > 0 && (now().seconds() - stamp > max_detection_age_s_ ||
    stamp - now().seconds() > max_detection_age_s_ ||
    (msg->tracker_session == tracker_session_ && stamp <= last_obstacle_stamp_))) return;
  if (msg->track_ids.size() != o.positions.size() || o.radii.size() != o.positions.size() ||
    o.classes.size() != o.positions.size() || o.confidences.size() != o.positions.size() ||
    msg->image_width == 0 || msg->image_height == 0) {
    publishCmd(0, 0);
    return;
  }
  if (!tracker_session_.empty() && tracker_session_ != msg->tracker_session) {
    // A restart is not evidence of passing. End old identity explicitly.
    if (state_ == ControllerState::APPROACHING || state_ == ControllerState::PASSING) {
      clearCurrentTarget();
      near_gate_ = false;
      transitionTo(ControllerState::SEARCHING);
      publishCmd(0, 0);
    }
    pending_candidate_ = PairDetection{};
    candidate_frames_ = 0;
    RCLCPP_WARN(get_logger(), "Tracker session changed: confirming a new candidate; no identity transfer");
  }
  last_obstacle_stamp_ = stamp;
  frame_ids_ = msg->track_ids;
  tracker_session_ = msg->tracker_session;
  frame_width_ = msg->image_width;
  frame_height_ = msg->image_height;
  obstacleCallback(std::make_shared<msgs::msg::ObstacleArray>(o));
}

bool BuoyCenteringNode::retired(const PairDetection & pair) const
{
  for (const auto & entry : retired_gates_) {
    if (now().seconds() - entry.time > passed_gate_suppression_time_s_) continue;
    const auto & old = entry.pair;
    // Suppress reuse of either constituent, including a new cross pair.
    if (!pair.session.empty() && pair.session == old.session &&
      ((pair.red_id >= 0 && pair.red_id == old.red_id) ||
       (pair.green_id >= 0 && pair.green_id == old.green_id))) return true;
    const double distance = std::hypot((pair.red_x + pair.green_x - old.red_x - old.green_x) / 2,
      (pair.red_y + pair.green_y - old.red_y - old.green_y) / 2);
    const double a = pair.red_radius + pair.green_radius;
    const double b = old.red_radius + old.green_radius;
    if (distance <= frame_width_ * retired_position_ratio_ && a > 0 && b > 0 &&
      std::max(a, b) / std::min(a, b) <= retired_radius_ratio_) return true;
  }
  return false;
}

bool BuoyCenteringNode::maneuverActive() const
{
  return state_ == ControllerState::CLEARING_GATE || state_ == ControllerState::TURNING ||
    state_ == ControllerState::TURN_STABILIZING;
}

void BuoyCenteringNode::controlTick()
{
  const double t = now().seconds();
  const double dt = std::clamp(t - tick_time_, 0.0, 0.1);
  tick_time_ = t;
  retired_gates_.erase(std::remove_if(retired_gates_.begin(), retired_gates_.end(),
    [&](const Retired & entry) {return t - entry.time > passed_gate_suppression_time_s_;}),
    retired_gates_.end());
  if (!maneuverActive()) {
    if (obstacle_received_ < 0 || t - obstacle_received_ > detection_timeout_s_) {
      publishCmd(0, 0);
      if (state_ == ControllerState::APPROACHING || state_ == ControllerState::PASSING) {
        clearCurrentTarget();
        near_gate_ = false;
        transitionTo(ControllerState::SEARCHING);
      }
      candidate_frames_ = 0;
      pending_candidate_ = PairDetection{};
      publishDiagnostic(false, 0, 0, 0, 0, 0, 0);
      publishTrackingDebug(frame_ids_.size());
    }
    return;
  }
  if (maneuver_fault_ || !has_heading_ || heading_received_ < 0 ||
    t - heading_received_ > heading_timeout_s_) {
    stable_samples_ = 0;
    publishCmd(0, 0);
    publishTrackingDebug(frame_ids_.size());
    return;
  }
  if (state_ == ControllerState::CLEARING_GATE) {
    if (!clear_heading_captured_) {
      clear_heading_ = heading_deg_;
      clear_heading_captured_ = true;
    }
    clearing_elapsed_ += dt;
    publishCmd(clear_forward_speed_, gate_logic::headingCommand(
      clear_heading_, heading_deg_, clear_heading_kp_, turn_angular_command_));
    if (clearing_elapsed_ >= (gate_count_ == 3 ? gate_3_clear_time_s_ : gate_7_clear_time_s_)) {
      turn_start_heading_ = heading_deg_;
      turn_target_heading_ = gate_logic::compassTarget(turn_start_heading_, turn_angle_deg_, course_layout_ == "A");
      maneuver_started_ = t;
      stable_samples_ = 0;
      transitionTo(ControllerState::TURNING);
      publishCmd(0, 0);
    }
  } else if (t - maneuver_started_ > turn_timeout_s_) {
    maneuver_fault_ = true;
    publishCmd(0, 0);
    RCLCPP_ERROR(get_logger(), "Turn timeout: latched STOP; inspect heading and restart guidance");
  } else if (state_ == ControllerState::TURN_STABILIZING) {
    publishCmd(0, gate_logic::headingCommand(turn_target_heading_, heading_deg_,
      clear_heading_kp_, turn_angular_command_));
    if (stable_samples_ >= turn_stable_samples_) {
      transitionTo(ControllerState::SEARCHING);
      candidate_frames_ = 0;
      pending_candidate_ = PairDetection{};
      publishCmd(0, 0);  // next fresh vision frame may acquire
    }
  } else {
    const double error = gate_logic::wrap(turn_target_heading_ - heading_deg_);
    publishCmd(turn_forward_speed_, error > 0 ? -turn_angular_command_ : turn_angular_command_);
  }
  publishDiagnostic(false, 0, commanded_linear_, commanded_angular_, 0, 0, 0);
  publishTrackingDebug(frame_ids_.size());
}

void BuoyCenteringNode::publishTrackingDebug(size_t count)
{
  std::ostringstream out;
  out << "state=" << stateName() << " gate_count=" << gate_count_
      << " acquisition=" << (state_ == ControllerState::SEARCHING ? "ON" : "OFF")
      << " detected=" << count << " session=" << tracker_session_ << " tracks=";
  for (auto id : frame_ids_) out << id << ',';
  out << " candidates=";
  for (const auto & p : debug_candidates_) out << p.red_id << '/' << p.green_id << ',';
  out << " locked=" << current_target_.red_id << '/' << current_target_.green_id
      << " score=" << selected_score_ << " confirm=" << candidate_frames_
      << " lost=" << approaching_target_lost_frames_ << '/' << lost_frames_
      << " visible_locked=" << visible_locked_
      << " partial_recovery=" << partial_recovery_
      << " recovery_missing=" << recovery_missing_
      << " switch_candidate=" << switch_candidate_.red_id << '/' << switch_candidate_.green_id
      << " switch_confirm=" << switch_confirm_frames_ << " switch_reason=" << switch_reason_
      << " pass_armed=" << near_gate_
      << " reassociated=" << reassociation_debug_ << " retired=";
  for (const auto & r : retired_gates_) out << r.pair.red_id << '/' << r.pair.green_id << ',';
  out << " clear_elapsed=" << clearing_elapsed_ << " turn_start=" << turn_start_heading_
      << " target_heading=" << turn_target_heading_ << " heading=" << heading_deg_
      << " heading_error=" << gate_logic::wrap(turn_target_heading_ - heading_deg_)
      << " stable=" << stable_samples_ << " fault=" << maneuver_fault_;
  std_msgs::msg::String msg;
  msg.data = out.str();
  tracking_debug_pub_->publish(msg);
}

void BuoyCenteringNode::publishDiagnostic(
  bool target_detected, double error_x, double linear_x, double angular_z,
  double p, double i, double d)
{
  msgs::msg::ControllerDiagnostic msg;
  msg.header.stamp = this->get_clock()->now();
  msg.controller_state = stateName();
  msg.mission_state = mission_state_;
  msg.target_detected = target_detected;
  msg.horizontal_error = error_x;
  msg.p_contribution = p;
  msg.i_contribution = i;
  msg.d_contribution = d;
  msg.linear_x = linear_x;
  msg.angular_z = angular_z;
  diagnostic_pub_->publish(msg);
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::BuoyCenteringNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
