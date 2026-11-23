#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <termios.h>
#include <vector>

namespace dummy_hardware_interface
{

class DummyAsciiTransport
{
public:
  DummyAsciiTransport() = default;
  ~DummyAsciiTransport();

  bool open_port(const std::string & port, int baud_rate);
  void close_port();
  bool is_open() const;

  void set_query_timeout(std::chrono::milliseconds timeout);
  void set_motion_ack_timeout(std::chrono::milliseconds timeout);
  void set_slow_command_timeout(std::chrono::milliseconds timeout);

  bool start();
  bool stop();
  bool home();
  bool reset();
  bool disable();
  bool set_home_current();
  bool calibration();
  bool set_command_mode(int mode);

  std::optional<std::array<double, 6>> get_home_offset_deg();
  std::optional<std::array<double, 6>> get_joint_positions_deg();
  std::optional<std::array<double, 3>> get_base_position_mm();
  std::optional<std::array<double, 6>> get_linear_pose_mm_deg();

  bool send_joint_target_deg(
    const std::array<double, 6> & joints_deg,
    std::optional<double> speed_deg = std::nullopt,
    char prefix = '>');
  bool send_pose_target_mm_deg(
    const std::array<double, 6> & pose_mm_deg,
    std::optional<double> speed = std::nullopt);

  bool send_raw_line(const std::string & line);
  bool flush_input();
  std::vector<std::string> read_available_lines(std::chrono::milliseconds max_wait);
  const std::vector<std::string> & last_received_lines() const;

private:
  bool send_simple_command(
    const std::string & cmd,
    const std::string & expected_substring,
    std::chrono::milliseconds timeout);
  std::optional<std::string> query_matching_line(
    const std::string & cmd,
    std::chrono::milliseconds timeout,
    const std::string & must_contain);
  static std::optional<int> parse_queue_space(const std::string & line);
  static std::string trim(const std::string & s);
  static bool parse_six_values(const std::string & line, std::array<double, 6> & values);
  static bool parse_three_values(const std::string & line, std::array<double, 3> & values);
  static speed_t baud_to_constant(int baud_rate);
  std::optional<std::string> read_line(std::chrono::milliseconds timeout);

  int fd_{-1};
  std::string port_name_;
  int baud_rate_{115200};
  std::chrono::milliseconds query_timeout_{300};
  std::chrono::milliseconds motion_ack_timeout_{300};
  std::chrono::milliseconds slow_command_timeout_{15000};
  std::vector<std::string> last_received_lines_;
};

}  // namespace dummy_hardware_interface
