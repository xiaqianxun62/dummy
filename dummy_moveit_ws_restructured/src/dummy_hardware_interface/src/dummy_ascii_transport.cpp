#include "dummy_hardware_interface/dummy_ascii_transport.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sstream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace dummy_hardware_interface
{

DummyAsciiTransport::~DummyAsciiTransport()
{
  close_port();
}

bool DummyAsciiTransport::open_port(const std::string & port, int baud_rate)
{
  close_port();

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close_port();
    return false;
  }

  cfmakeraw(&tty);
  const speed_t speed = baud_to_constant(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close_port();
    return false;
  }

  port_name_ = port;
  baud_rate_ = baud_rate;
  tcflush(fd_, TCIOFLUSH);
  return true;
}

void DummyAsciiTransport::close_port()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool DummyAsciiTransport::is_open() const
{
  return fd_ >= 0;
}

void DummyAsciiTransport::set_query_timeout(std::chrono::milliseconds timeout)
{
  query_timeout_ = timeout;
}

void DummyAsciiTransport::set_motion_ack_timeout(std::chrono::milliseconds timeout)
{
  motion_ack_timeout_ = timeout;
}

void DummyAsciiTransport::set_slow_command_timeout(std::chrono::milliseconds timeout)
{
  slow_command_timeout_ = timeout;
}

bool DummyAsciiTransport::send_raw_line(const std::string & line)
{
  if (!is_open()) {
    return false;
  }

  std::string payload = line;
  if (payload.size() < 2 || payload.substr(payload.size() - 2) != "\r\n") {
    payload += "\r\n";
  }

  const auto * data = payload.data();
  size_t total = 0;
  while (total < payload.size()) {
    const ssize_t n = ::write(fd_, data + total, payload.size() - total);
    if (n > 0) {
      total += static_cast<size_t>(n);
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      fd_set writefds;
      FD_ZERO(&writefds);
      FD_SET(fd_, &writefds);
      timeval tv{};
      tv.tv_sec = 0;
      tv.tv_usec = 200000;
      const int ret = select(fd_ + 1, nullptr, &writefds, nullptr, &tv);
      if (ret <= 0) {
        return false;
      }
      continue;
    }
    return false;
  }
  return true;
}

bool DummyAsciiTransport::flush_input()
{
  if (!is_open()) {
    return false;
  }
  return tcflush(fd_, TCIFLUSH) == 0;
}

std::optional<std::string> DummyAsciiTransport::read_line(std::chrono::milliseconds timeout)
{
  if (!is_open()) {
    return std::nullopt;
  }

  std::string line;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now);
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    timeval tv{};
    tv.tv_sec = static_cast<long>(remaining.count() / 1000);
    tv.tv_usec = static_cast<long>((remaining.count() % 1000) * 1000);
    const int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      return std::nullopt;
    }
    if (ret == 0) {
      break;
    }

    char ch = 0;
    const ssize_t n = ::read(fd_, &ch, 1);
    if (n <= 0) {
      continue;
    }
    if (ch == '\n') {
      return trim(line);
    }
    if (ch != '\r') {
      line.push_back(ch);
    }
  }

  if (!line.empty()) {
    return trim(line);
  }
  return std::nullopt;
}

std::vector<std::string> DummyAsciiTransport::read_available_lines(std::chrono::milliseconds max_wait)
{
  std::vector<std::string> lines;
  const auto deadline = std::chrono::steady_clock::now() + max_wait;
  while (std::chrono::steady_clock::now() < deadline) {
    auto line = read_line(std::chrono::milliseconds(20));
    if (!line) {
      break;
    }
    if (!line->empty()) {
      lines.push_back(*line);
    }
  }
  return lines;
}

const std::vector<std::string> & DummyAsciiTransport::last_received_lines() const
{
  return last_received_lines_;
}

bool DummyAsciiTransport::send_simple_command(
  const std::string & cmd,
  const std::string & expected_substring,
  std::chrono::milliseconds timeout)
{
  flush_input();
  if (!send_raw_line(cmd)) {
    return false;
  }
  if (expected_substring.empty()) {
    return true;
  }
  auto line = query_matching_line("", timeout, expected_substring);
  return line.has_value();
}

std::optional<std::string> DummyAsciiTransport::query_matching_line(
  const std::string & cmd,
  std::chrono::milliseconds timeout,
  const std::string & must_contain)
{
  last_received_lines_.clear();

  if (!cmd.empty()) {
    flush_input();
    if (!send_raw_line(cmd)) {
      return std::nullopt;
    }
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto line = read_line(std::chrono::milliseconds(50));
    if (!line) {
      continue;
    }
    last_received_lines_.push_back(*line);
    if (must_contain.empty() || line->find(must_contain) != std::string::npos) {
      return line;
    }
  }
  return std::nullopt;
}

std::optional<int> DummyAsciiTransport::parse_queue_space(const std::string & line)
{
  std::stringstream ss(line);
  int value = -1;
  ss >> value;
  if (!ss.fail()) {
    return value;
  }
  return std::nullopt;
}

bool DummyAsciiTransport::parse_six_values(const std::string & line, std::array<double, 6> & values)
{
  return std::sscanf(
           line.c_str(), "ok %lf %lf %lf %lf %lf %lf", &values[0], &values[1], &values[2], &values[3],
           &values[4], &values[5]) == 6;
}

bool DummyAsciiTransport::parse_three_values(const std::string & line, std::array<double, 3> & values)
{
  return std::sscanf(line.c_str(), "ok %lf %lf %lf", &values[0], &values[1], &values[2]) == 3;
}

std::string DummyAsciiTransport::trim(const std::string & s)
{
  const auto begin = s.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return {};
  }
  const auto end = s.find_last_not_of(" \t\r\n");
  return s.substr(begin, end - begin + 1);
}

speed_t DummyAsciiTransport::baud_to_constant(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
    default:
      return B115200;
  }
}

bool DummyAsciiTransport::start()
{
  return send_simple_command("!START", "Started ok", query_timeout_);
}

bool DummyAsciiTransport::stop()
{
  return send_simple_command("!STOP", "Stopped ok", query_timeout_);
}

bool DummyAsciiTransport::home()
{
  return send_simple_command("!HOME", "Started ok", slow_command_timeout_);
}

bool DummyAsciiTransport::reset()
{
  return send_simple_command("!RESET", "Started ok", slow_command_timeout_);
}

bool DummyAsciiTransport::disable()
{
  return send_simple_command("!DISABLE", "Disabled ok", query_timeout_);
}

bool DummyAsciiTransport::set_home_current()
{
  return send_simple_command("!SET_HOME_CURRENT", "SET_HOME_CURRENT", query_timeout_);
}

bool DummyAsciiTransport::calibration()
{
  return send_simple_command("!CALIBRATION", "calibration ok", slow_command_timeout_);
}

bool DummyAsciiTransport::set_command_mode(int mode)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "#CMDMODE %d", mode);
  return send_simple_command(buffer, "command mode", query_timeout_);
}

std::optional<std::array<double, 6>> DummyAsciiTransport::get_home_offset_deg()
{
  auto line = query_matching_line("#GET_HOME_OFFSET", query_timeout_, "ok ");
  if (!line) {
    return std::nullopt;
  }
  std::array<double, 6> values{};
  if (!parse_six_values(*line, values)) {
    return std::nullopt;
  }
  return values;
}

std::optional<std::array<double, 6>> DummyAsciiTransport::get_joint_positions_deg()
{
  auto line = query_matching_line("#GETJPOS", query_timeout_, "ok ");
  if (!line) {
    return std::nullopt;
  }
  std::array<double, 6> values{};
  if (!parse_six_values(*line, values)) {
    return std::nullopt;
  }
  return values;
}

std::optional<std::array<double, 3>> DummyAsciiTransport::get_base_position_mm()
{
  auto line = query_matching_line("#GET_BASE_POS", query_timeout_, "ok ");
  if (!line) {
    return std::nullopt;
  }
  std::array<double, 3> values{};
  if (!parse_three_values(*line, values)) {
    return std::nullopt;
  }
  return values;
}

std::optional<std::array<double, 6>> DummyAsciiTransport::get_linear_pose_mm_deg()
{
  auto line = query_matching_line("#GETLPOS", query_timeout_, "ok ");
  if (!line) {
    return std::nullopt;
  }
  std::array<double, 6> values{};
  if (!parse_six_values(*line, values)) {
    return std::nullopt;
  }
  return values;
}

bool DummyAsciiTransport::send_joint_target_deg(
  const std::array<double, 6> & joints_deg,
  std::optional<double> speed_deg,
  char prefix)
{
  char buffer[256];
  if (speed_deg.has_value()) {
    std::snprintf(
      buffer, sizeof(buffer), "%c%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", prefix, joints_deg[0], joints_deg[1],
      joints_deg[2], joints_deg[3], joints_deg[4], joints_deg[5], *speed_deg);
  } else {
    std::snprintf(
      buffer, sizeof(buffer), "%c%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", prefix, joints_deg[0], joints_deg[1],
      joints_deg[2], joints_deg[3], joints_deg[4], joints_deg[5]);
  }

  flush_input();
  if (!send_raw_line(buffer)) {
    return false;
  }
  auto line = query_matching_line("", motion_ack_timeout_, "");
  return line.has_value() && parse_queue_space(*line).has_value();
}

bool DummyAsciiTransport::send_pose_target_mm_deg(
  const std::array<double, 6> & pose_mm_deg,
  std::optional<double> speed)
{
  char buffer[256];
  if (speed.has_value()) {
    std::snprintf(
      buffer, sizeof(buffer), "@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", pose_mm_deg[0], pose_mm_deg[1],
      pose_mm_deg[2], pose_mm_deg[3], pose_mm_deg[4], pose_mm_deg[5], *speed);
  } else {
    std::snprintf(
      buffer, sizeof(buffer), "@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f", pose_mm_deg[0], pose_mm_deg[1],
      pose_mm_deg[2], pose_mm_deg[3], pose_mm_deg[4], pose_mm_deg[5]);
  }

  flush_input();
  if (!send_raw_line(buffer)) {
    return false;
  }
  auto line = query_matching_line("", motion_ack_timeout_, "");
  return line.has_value() && parse_queue_space(*line).has_value();
}

}  // namespace dummy_hardware_interface
