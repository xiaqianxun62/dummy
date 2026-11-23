#include "dummy_hardware_interface/dummy_ascii_transport.hpp"

#include <array>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace
{

void print_recent_serial_lines(const dummy_hardware_interface::DummyAsciiTransport & transport)
{
  const auto & lines = transport.last_received_lines();
  if (lines.empty()) {
    std::cerr << "Recent serial lines: <none>" << std::endl;
    return;
  }

  std::cerr << "Recent serial lines:" << std::endl;
  for (const auto & line : lines) {
    std::cerr << "  " << line << std::endl;
  }
}

void print_usage()
{
  std::cerr << "Usage:\n"
            << "  dummy_ascii_cli [--port /dev/ttyACM0] [--baud 115200] <command> [args...]\n\n"
            << "Commands:\n"
            << "  start | stop | home | reset | disable | set-home-current | calibration\n"
            << "  cmdmode <mode>\n"
            << "  getjpos | get-home-offset | get-base-pos | getlpos\n"
            << "  sendj j1 j2 j3 j4 j5 j6 [speed_deg]\n"
            << "  sendl x y z a b c [speed]\n";
}

template <std::size_t N>
std::optional<std::array<double, N>> parse_values(const std::vector<std::string> & args, std::size_t start)
{
  if (args.size() < start + N) {
    return std::nullopt;
  }
  std::array<double, N> values{};
  for (std::size_t i = 0; i < N; ++i) {
    try {
      values[i] = std::stod(args[start + i]);
    } catch (...) {
      return std::nullopt;
    }
  }
  return values;
}

template <std::size_t N>
void print_values(const std::array<double, N> & values)
{
  for (std::size_t i = 0; i < N; ++i) {
    if (i > 0) {
      std::cout << ' ';
    }
    std::cout << values[i];
  }
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char ** argv)
{
  std::string port = "/dev/ttyACM0";
  int baud = 115200;
  std::vector<std::string> positional;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--port" && i + 1 < argc) {
      port = argv[++i];
    } else if (arg == "--baud" && i + 1 < argc) {
      baud = std::stoi(argv[++i]);
    } else {
      positional.push_back(arg);
    }
  }

  if (positional.empty()) {
    print_usage();
    return 1;
  }

  dummy_hardware_interface::DummyAsciiTransport transport;
  transport.set_query_timeout(std::chrono::milliseconds(500));
  transport.set_motion_ack_timeout(std::chrono::milliseconds(500));
  transport.set_slow_command_timeout(std::chrono::milliseconds(20000));

  if (!transport.open_port(port, baud)) {
    std::cerr << "Failed to open serial port: " << port << std::endl;
    return 1;
  }

  const std::string cmd = positional[0];
  bool ok = false;

  if (cmd == "start") {
    ok = transport.start();
  } else if (cmd == "stop") {
    ok = transport.stop();
  } else if (cmd == "home") {
    ok = transport.home();
  } else if (cmd == "reset") {
    ok = transport.reset();
  } else if (cmd == "disable") {
    ok = transport.disable();
  } else if (cmd == "set-home-current") {
    ok = transport.set_home_current();
  } else if (cmd == "calibration") {
    ok = transport.calibration();
  } else if (cmd == "cmdmode") {
    if (positional.size() < 2) {
      print_usage();
      return 1;
    }
    ok = transport.set_command_mode(std::stoi(positional[1]));
  } else if (cmd == "getjpos") {
    auto values = transport.get_joint_positions_deg();
    if (!values) {
      std::cerr << "No valid #GETJPOS response" << std::endl;
      return 1;
    }
    print_values(*values);
    return 0;
  } else if (cmd == "get-home-offset") {
    auto values = transport.get_home_offset_deg();
    if (!values) {
      std::cerr << "No valid #GET_HOME_OFFSET response" << std::endl;
      return 1;
    }
    print_values(*values);
    return 0;
  } else if (cmd == "get-base-pos") {
    auto values = transport.get_base_position_mm();
    if (!values) {
      std::cerr << "No valid #GET_BASE_POS response" << std::endl;
      return 1;
    }
    print_values(*values);
    return 0;
  } else if (cmd == "getlpos") {
    auto values = transport.get_linear_pose_mm_deg();
    if (!values) {
      std::cerr << "No valid #GETLPOS response" << std::endl;
      return 1;
    }
    print_values(*values);
    return 0;
  } else if (cmd == "sendj") {
    auto values = parse_values<6>(positional, 1);
    if (!values) {
      print_usage();
      return 1;
    }
    std::optional<double> speed = std::nullopt;
    if (positional.size() >= 8) {
      speed = std::stod(positional[7]);
    }
    ok = transport.send_joint_target_deg(*values, speed);
  } else if (cmd == "sendl") {
    auto values = parse_values<6>(positional, 1);
    if (!values) {
      print_usage();
      return 1;
    }
    std::optional<double> speed = std::nullopt;
    if (positional.size() >= 8) {
      speed = std::stod(positional[7]);
    }
    ok = transport.send_pose_target_mm_deg(*values, speed);
  } else {
    print_usage();
    return 1;
  }

  if (!ok) {
    std::cerr << "Command failed: " << cmd << std::endl;
    print_recent_serial_lines(transport);
    return 1;
  }

  std::cout << "ok" << std::endl;
  return 0;
}
