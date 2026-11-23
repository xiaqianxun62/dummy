#pragma once

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include "dummy_hardware_interface/dummy_ascii_transport.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace dummy_hardware_interface
{

class DummyHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DummyHardwareInterface)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static std::array<double, 6> parse_array_parameter(
    const std::string & csv,
    const std::array<double, 6> & fallback);
  std::array<double, 6> ros_to_firmware_deg(const std::vector<double> & ros_rad) const;
  std::array<double, 6> firmware_deg_to_ros(const std::array<double, 6> & fw_deg) const;

  DummyAsciiTransport transport_;
  std::string serial_port_{"/dev/ttyACM0"};
  int baud_rate_{115200};
  int command_mode_{2};
  double read_timeout_ms_{200.0};
  double slow_command_timeout_ms_{15000.0};
  double write_rate_hz_{20.0};
  double joint_speed_deg_{0.0};
  bool start_on_activate_{false};
  bool disable_on_deactivate_{true};
  bool auto_home_on_activate_{false};
  char joint_command_prefix_{'>'};

  std::array<double, 6> position_offsets_rad_{{0.0, 0.0, 1.57079, 0.0, 0.0, 0.0}};
  std::array<double, 6> position_signs_{{1.0, 1.0, 1.0, 1.0, -1.0, -1.0}};

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> last_sent_commands_;
  std::chrono::steady_clock::time_point last_write_tp_{};
  bool first_read_{true};
};

}  // namespace dummy_hardware_interface
