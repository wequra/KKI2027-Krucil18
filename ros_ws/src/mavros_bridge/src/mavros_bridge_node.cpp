#include "mavros_bridge/mavros_bridge_node.hpp"
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace dagozilla
{

MavrosBridgeNode::MavrosBridgeNode()
  : rclcpp::Node("mavros_bridge_node")
{
  this->declare_parameter<double>("command_timeout_sec", 0.75);
  this->declare_parameter<bool>("configure_mavros_velocity_frame", true);
  this->declare_parameter<std::string>("mavros_velocity_frame", "BODY_NED");
  command_timeout_sec_ = std::max(0.1, this->get_parameter("command_timeout_sec").as_double());
  mavros_velocity_frame_ = this->get_parameter("mavros_velocity_frame").as_string();

  auto qos = rclcpp::SensorDataQoS();
  cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

#ifdef HAVE_MAVROS_MSGS
  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", 10,
    std::bind(&MavrosBridgeNode::stateCallback, this, std::placeholders::_1));

  mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
    "/mavros/set_mode", rmw_qos_profile_services_default, cb_group_);
#endif

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", qos,
    std::bind(&MavrosBridgeNode::gpsCallback, this, std::placeholders::_1));

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/asv/cmd_vel", 10,
    std::bind(&MavrosBridgeNode::cmdVelCallback, this, std::placeholders::_1));

  mavros_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "/mavros/setpoint_velocity/cmd_vel_unstamped", 10);

  set_mode_srv_ = this->create_service<msgs::srv::SetMode>(
    "/asv/set_mode",
    std::bind(&MavrosBridgeNode::onSetMode, this, std::placeholders::_1, std::placeholders::_2),
    rclcpp::ServicesQoS(), cb_group_);

  last_cmd_time_ = std::chrono::steady_clock::now();
  command_timer_ = this->create_wall_timer(100ms, std::bind(&MavrosBridgeNode::publishCommand, this));

  if (this->get_parameter("configure_mavros_velocity_frame").as_bool()) {
    mavros_velocity_params_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "/mavros/setpoint_velocity");
    mavros_frame_timer_ = this->create_wall_timer(
      1s, std::bind(&MavrosBridgeNode::configureMavrosVelocityFrame, this));
  }

  RCLCPP_INFO(
    this->get_logger(),
    "mavros_bridge_node siap & forwarding /asv/cmd_vel ke MAVROS "
    "(10 Hz, timeout %.2f s).", command_timeout_sec_);
}

#ifdef HAVE_MAVROS_MSGS
void MavrosBridgeNode::stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  current_state_ = *msg;
}
#endif

void MavrosBridgeNode::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  current_gps_ = *msg;
}

void MavrosBridgeNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel_ = *msg;
  last_cmd_time_ = std::chrono::steady_clock::now();
  has_command_ = true;
}

void MavrosBridgeNode::publishCommand()
{
  geometry_msgs::msg::Twist command;
  const auto age = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - last_cmd_time_).count();

  if (has_command_ && age <= command_timeout_sec_) {
    command = last_cmd_vel_;
  }

  // Startup and stale-input behavior are both an explicit zero command.
  mavros_cmd_vel_pub_->publish(command);
}

void MavrosBridgeNode::configureMavrosVelocityFrame()
{
  if (mavros_frame_configured_ || mavros_frame_request_in_flight_ ||
    !mavros_velocity_params_->service_is_ready())
  {
    return;
  }

  mavros_frame_request_in_flight_ = true;
  mavros_velocity_params_->set_parameters(
    {rclcpp::Parameter("mav_frame", mavros_velocity_frame_)},
    [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
      mavros_frame_request_in_flight_ = false;
      try {
        const auto results = future.get();
        if (!results.empty() && results.front().successful) {
          mavros_frame_configured_ = true;
          mavros_frame_timer_->cancel();
          RCLCPP_INFO(
            this->get_logger(), "MAVROS setpoint_velocity mav_frame=%s.",
            mavros_velocity_frame_.c_str());
          return;
        }

        const std::string reason = results.empty() ? "empty response" : results.front().reason;
        RCLCPP_WARN(
          this->get_logger(), "Gagal mengatur MAVROS velocity frame: %s; akan mencoba lagi.",
          reason.c_str());
      } catch (const std::exception & error) {
        RCLCPP_WARN(
          this->get_logger(), "Error saat mengatur MAVROS velocity frame: %s; akan mencoba lagi.",
          error.what());
      }
    });
}

void MavrosBridgeNode::onSetMode(
  const std::shared_ptr<msgs::srv::SetMode::Request> request,
  std::shared_ptr<msgs::srv::SetMode::Response> response)
{
#ifdef HAVE_MAVROS_MSGS
  if (!mode_client_->wait_for_service(2s)) {
    response->success = false;
    response->message = "mavros/set_mode service tidak tersedia";
    return;
  }

  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->custom_mode = request->mode;  // contoh: "GUIDED", "AUTO", "HOLD", "RTL"

  auto future = mode_client_->async_send_request(req);
  if (future.wait_for(3s) != std::future_status::ready) {
    response->success = false;
    response->message = "Timeout menunggu respons Pixhawk";
    return;
  }

  auto result = future.get();
  if (result->mode_sent) {
    response->success = true;
    response->message = "Mode diubah ke " + request->mode;
  } else {
    response->success = false;
    response->message = "Gagal mengubah mode di Pixhawk";
  }
#else
  response->success = true;
  response->message = "Mode " + request->mode + " diset (mock mode tanpa mavros_msgs)";
#endif
}

}  // namespace dagozilla

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dagozilla::MavrosBridgeNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
