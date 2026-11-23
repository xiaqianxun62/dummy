#include "dummy_hardware_interface/dummy_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

#include "hardware_interface/handle.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double deg2rad(double deg) { return deg * kPi / 180.0; }
constexpr double rad2deg(double rad) { return rad * 180.0 / kPi; }
}  // namespace

namespace dummy_hardware_interface
{

hardware_interface::CallbackReturn DummyHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t joint_count = info_.joints.size();
  if (joint_count != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Expected 6 joints, got %zu", joint_count);
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  last_sent_commands_.assign(joint_count, std::numeric_limits<double>::quiet_NaN());

  auto get_param = [this](const std::string & key, const std::string & fallback) {
    const auto it = info_.hardware_parameters.find(key);
    return it == info_.hardware_parameters.end() ? fallback : it->second;
  };

  serial_port_ = get_param("serial_port", "/dev/ttyACM0");
  baud_rate_ = std::stoi(get_param("baud_rate", "115200"));
  command_mode_ = std::stoi(get_param("command_mode", "2"));
  read_timeout_ms_ = std::stod(get_param("read_timeout_ms", "200.0"));
  slow_command_timeout_ms_ = std::stod(get_param("slow_command_timeout_ms", "15000.0"));
  write_rate_hz_ = std::stod(get_param("write_rate_hz", "20.0"));
  joint_speed_deg_ = std::stod(get_param("joint_speed_deg", "0.0"));
  start_on_activate_ = get_param("start_on_activate", "false") == "true";
  disable_on_deactivate_ = get_param("disable_on_deactivate", "true") == "true";
  auto_home_on_activate_ = get_param("auto_home_on_activate", "false") == "true";
  const auto prefix_str = get_param("joint_command_prefix", ">");
  joint_command_prefix_ = prefix_str.empty() ? '>' : prefix_str.front();

  position_offsets_rad_ = parse_array_parameter(
    get_param("position_offsets_rad", "0,0,1.57079,0,0,0"),
    {0.0, 0.0, 1.57079, 0.0, 0.0, 0.0});
  position_signs_ = parse_array_parameter(
    get_param("position_signs", "1,1,1,1,-1,-1"),
    {1.0, 1.0, 1.0, 1.0, -1.0, -1.0});

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DummyHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  states.reserve(info_.joints.size() * 2);
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    states.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> DummyHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  commands.reserve(info_.joints.size());
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    commands.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return commands;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!transport_.open_port(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("DummyHardwareInterface"), "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  transport_.set_query_timeout(std::chrono::milliseconds(static_cast<int>(read_timeout_ms_)));
  transport_.set_motion_ack_timeout(std::chrono::milliseconds(static_cast<int>(read_timeout_ms_)));
  transport_.set_slow_command_timeout(std::chrono::milliseconds(static_cast<int>(slow_command_timeout_ms_)));

  if (start_on_activate_ && !transport_.start()) {
    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to send !START");
    transport_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!transport_.set_command_mode(command_mode_)) {
    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to set command mode to %d", command_mode_);
    transport_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto_home_on_activate_ && !transport_.home()) {
    RCLCPP_ERROR(rclcpp::get_logger("DummyHardwareInterface"), "Failed to complete !HOME during activation");
    transport_.close_port();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (auto home = transport_.get_home_offset_deg()) {
    RCLCPP_INFO(
      rclcpp::get_logger("DummyHardwareInterface"),
      "Firmware semantic home offset [deg]: %.2f %.2f %.2f %.2f %.2f %.2f",
      (*home)[0], (*home)[1], (*home)[2], (*home)[3], (*home)[4], (*home)[5]);
  }

  if (auto joints_deg = transport_.get_joint_positions_deg()) {
    const auto ros = firmware_deg_to_ros(*joints_deg);
    for (size_t i = 0; i < ros.size(); ++i) {
      hw_positions_[i] = ros[i];
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = ros[i];
      last_sent_commands_[i] = ros[i];
    }
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("DummyHardwareInterface"),
      "Activation succeeded but initial #GETJPOS did not return valid data.");
  }

  first_read_ = true;
  last_write_tp_ = std::chrono::steady_clock::now();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DummyHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (disable_on_deactivate_) {
    transport_.stop();
    transport_.disable();
  }
  transport_.close_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyHardwareInterface::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  auto joints_deg = transport_.get_joint_positions_deg();
  if (!joints_deg) {
    return hardware_interface::return_type::OK;
  }

  const auto ros = firmware_deg_to_ros(*joints_deg);
  const double dt = std::max(period.seconds(), 1e-6);
  for (size_t i = 0; i < ros.size(); ++i) {
    const double previous = hw_positions_[i];
    hw_positions_[i] = ros[i];
    hw_velocities_[i] = first_read_ ? 0.0 : (hw_positions_[i] - previous) / dt;
  }
  first_read_ = false;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  const double min_interval = write_rate_hz_ > 0.0 ? 1.0 / write_rate_hz_ : 0.0;
  if (min_interval > 0.0) {
    const double elapsed = std::chrono::duration<double>(now - last_write_tp_).count();
    if (elapsed < min_interval) {
      return hardware_interface::return_type::OK;
    }
  }

  bool changed = false;
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    if (!std::isfinite(last_sent_commands_[i]) || std::fabs(hw_commands_[i] - last_sent_commands_[i]) > 1e-4) {
      changed = true;
      break;
    }
  }
  if (!changed) {
    return hardware_interface::return_type::OK;
  }

  const auto fw_deg = ros_to_firmware_deg(hw_commands_);
  const std::optional<double> speed = joint_speed_deg_ > 0.0 ? std::optional<double>(joint_speed_deg_) : std::nullopt;
  if (transport_.send_joint_target_deg(fw_deg, speed, joint_command_prefix_)) {
    last_sent_commands_ = hw_commands_;
    last_write_tp_ = now;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("DummyHardwareInterface"), "Joint command was not acknowledged by firmware.");
  }
  return hardware_interface::return_type::OK;
}

std::array<double, 6> DummyHardwareInterface::parse_array_parameter(
  const std::string & csv,
  const std::array<double, 6> & fallback)
{
  std::array<double, 6> values = fallback;
  std::stringstream ss(csv);
  std::string token;
  size_t i = 0;
  while (std::getline(ss, token, ',') && i < values.size()) {
    try {
      values[i++] = std::stod(token);
    } catch (...) {
      return fallback;
    }
  }
  return values;
}

std::array<double, 6> DummyHardwareInterface::ros_to_firmware_deg(const std::vector<double> & ros_rad) const
{
  std::array<double, 6> out{};
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = rad2deg((ros_rad[i] + position_offsets_rad_[i]) * position_signs_[i]);
  }
  return out;
}

std::array<double, 6> DummyHardwareInterface::firmware_deg_to_ros(const std::array<double, 6> & fw_deg) const
{
  std::array<double, 6> out{};
  for (size_t i = 0; i < out.size(); ++i) {
    out[i] = deg2rad(fw_deg[i]) * position_signs_[i] - position_offsets_rad_[i];
  }
  return out;
}

}  // namespace dummy_hardware_interface

PLUGINLIB_EXPORT_CLASS(dummy_hardware_interface::DummyHardwareInterface, hardware_interface::SystemInterface)
