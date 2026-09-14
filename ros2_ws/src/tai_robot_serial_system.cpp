// Copyright 2026 TAI Robot One contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tai_robot_one/tai_robot_serial_system.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <locale>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace tai_robot_one
{
namespace
{

using SteadyClock = std::chrono::steady_clock;

constexpr std::array<const char *, 4> kWheelJointNames = {
  "front_left_wheel_joint",
  "front_right_wheel_joint",
  "rear_left_wheel_joint",
  "rear_right_wheel_joint",
};
constexpr const char * kLiftJointName = "lift_joint";
constexpr double kDefaultMaxWheelSpeedRadS = 0.5 / 0.0875;
constexpr double kDefaultMaxLiftPositionM = 0.520;
constexpr uint16_t kBaseProtocolVersion = 1;
constexpr uint16_t kBaseFaultCommandTimeout = 1U << 0;
constexpr uint16_t kBaseFaultSoftwareEstop = 1U << 1;
constexpr uint16_t kBaseStatusEnabled = 1U << 0;

std::string interfaceName(const std::string & joint, const char * interface)
{
  return joint + "/" + interface;
}

std::vector<std::string> split(const std::string & input, char delimiter)
{
  std::vector<std::string> fields;
  std::size_t begin = 0;
  while (true) {
    const std::size_t end = input.find(delimiter, begin);
    fields.emplace_back(input.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1;
  }
  return fields;
}

bool parseUint32(const std::string & text, uint32_t & value)
{
  if (text.empty() || text.front() == '-') {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const uint64_t parsed = std::strtoull(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
    parsed > std::numeric_limits<uint32_t>::max())
  {
    return false;
  }
  value = static_cast<uint32_t>(parsed);
  return true;
}

bool parseUint16(const std::string & text, uint16_t & value)
{
  uint32_t parsed = 0;
  if (!parseUint32(text, parsed) || parsed > std::numeric_limits<uint16_t>::max()) {
    return false;
  }
  value = static_cast<uint16_t>(parsed);
  return true;
}

bool parseDouble(const std::string & text, double & value)
{
  if (text.empty()) {
    return false;
  }
  const char * const begin = text.data();
  const char * const end = begin + text.size();
  const auto result = std::from_chars(begin, end, value, std::chars_format::general);
  return result.ec == std::errc{} && result.ptr == end && std::isfinite(value);
}

bool parseFlag(const std::string & text, bool & value)
{
  if (text == "0") {
    value = false;
    return true;
  }
  if (text == "1") {
    value = true;
    return true;
  }
  return false;
}

bool sequenceNewer(uint32_t candidate, uint32_t previous)
{
  return static_cast<int32_t>(candidate - previous) > 0;
}

uint16_t crc16CcittFalse(const std::string & payload)
{
  uint16_t crc = 0xFFFF;
  for (const unsigned char byte : payload) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (unsigned int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x8000U) != 0U ?
        static_cast<uint16_t>((crc << 1) ^ 0x1021U) :
        static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::string frameBasePayload(const std::string & payload)
{
  char crc_text[5];
  std::snprintf(crc_text, sizeof(crc_text), "%04X", crc16CcittFalse(payload));
  return "@" + payload + "*" + crc_text + "\n";
}

bool decodeBaseFrame(const std::string & line, std::string & payload)
{
  if (line.size() < 7 || line.front() != '@') {
    return false;
  }
  const std::size_t star = line.rfind('*');
  if (star == std::string::npos || star + 5 != line.size()) {
    return false;
  }
  errno = 0;
  char * end = nullptr;
  const std::string crc_text = line.substr(star + 1);
  const auto received = std::strtoul(crc_text.c_str(), &end, 16);
  if (errno != 0 || end != crc_text.c_str() + crc_text.size() || received > 0xFFFFUL) {
    return false;
  }
  payload = line.substr(1, star - 1);
  return crc16CcittFalse(payload) == static_cast<uint16_t>(received);
}

speed_t baudConstant(unsigned int baud)
{
  switch (baud) {
    case 115200:
      return B115200;
#ifdef B460800
    case 460800:
      return B460800;
#endif
    default:
      return 0;
  }
}

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {closePort();}

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool openPort(const std::string & path, unsigned int baud, std::string & error)
  {
    closePort();
    const speed_t speed = baudConstant(baud);
    if (speed == 0) {
      error = "unsupported baud rate " + std::to_string(baud);
      return false;
    }

    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      error = "cannot open " + path + ": " + std::strerror(errno);
      return false;
    }

#ifdef TIOCEXCL
    // Do not let an old bridge or a second controller steal bytes from this
    // protocol after ros2_control has claimed the device.
    if (::ioctl(fd_, TIOCEXCL) != 0) {
      error = "cannot claim exclusive access to " + path + ": " + std::strerror(errno);
      closePort();
      return false;
    }
#endif

    termios options{};
    if (::tcgetattr(fd_, &options) != 0) {
      error = "tcgetattr(" + path + ") failed: " + std::strerror(errno);
      closePort();
      return false;
    }
    ::cfmakeraw(&options);
    options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
#ifdef CRTSCTS
    options.c_cflag &= ~CRTSCTS;
#endif
    options.c_cflag |= CS8 | CLOCAL | CREAD;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;
    if (::cfsetispeed(&options, speed) != 0 || ::cfsetospeed(&options, speed) != 0 ||
      ::tcsetattr(fd_, TCSANOW, &options) != 0)
    {
      error = "cannot configure " + path + ": " + std::strerror(errno);
      closePort();
      return false;
    }

    (void)::tcflush(fd_, TCIOFLUSH);
    path_ = path;
    rx_buffer_.clear();
    return true;
  }

  void closePort()
  {
    if (fd_ >= 0) {
      (void)::close(fd_);
    }
    fd_ = -1;
    path_.clear();
    rx_buffer_.clear();
  }

  bool isOpen() const {return fd_ >= 0;}

  bool healthy(std::string & error) const
  {
    if (fd_ < 0) {
      error = "serial port is closed";
      return false;
    }
    pollfd descriptor{fd_, 0, 0};
    const int result = ::poll(&descriptor, 1, 0);
    if (result < 0 && errno != EINTR) {
      error = "poll(" + path_ + ") failed: " + std::strerror(errno);
      return false;
    }
    if (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      error = "serial device disconnected: " + path_;
      return false;
    }
    return true;
  }

  bool writeLine(const std::string & line, std::string & error)
  {
    if (fd_ < 0) {
      error = "cannot write: serial port is closed";
      return false;
    }
    std::size_t offset = 0;
    while (offset < line.size()) {
      const ssize_t written = ::write(fd_, line.data() + offset, line.size() - offset);
      if (written > 0) {
        offset += static_cast<std::size_t>(written);
        continue;
      }
      if (written < 0 && errno == EINTR) {
        continue;
      }
      if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        pollfd descriptor{fd_, POLLOUT, 0};
        const int result = ::poll(&descriptor, 1, 5);
        if (result > 0 && (descriptor.revents & POLLOUT) != 0) {
          continue;
        }
      }
      error = "write(" + path_ + ") failed: " + std::strerror(errno);
      return false;
    }
    return true;
  }

  bool readLines(std::vector<std::string> & lines, std::string & error)
  {
    if (!healthy(error)) {
      return false;
    }

    while (true) {
      pollfd descriptor{fd_, POLLIN, 0};
      const int poll_result = ::poll(&descriptor, 1, 0);
      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        error = "poll(" + path_ + ") failed: " + std::strerror(errno);
        return false;
      }
      if (poll_result == 0 || (descriptor.revents & POLLIN) == 0) {
        break;
      }
      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        error = "serial device disconnected: " + path_;
        return false;
      }

      char buffer[512];
      const ssize_t count = ::read(fd_, buffer, sizeof(buffer));
      if (count > 0) {
        rx_buffer_.append(buffer, static_cast<std::size_t>(count));
      } else if (count == 0) {
        error = "serial device returned EOF: " + path_;
        return false;
      } else if (errno == EINTR) {
        continue;
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        break;
      } else {
        error = "read(" + path_ + ") failed: " + std::strerror(errno);
        return false;
      }
    }

    if (rx_buffer_.size() > 4096 && rx_buffer_.find('\n') == std::string::npos) {
      error = "oversize serial record from " + path_;
      return false;
    }
    while (true) {
      const std::size_t newline = rx_buffer_.find('\n');
      if (newline == std::string::npos) {
        break;
      }
      std::string line = rx_buffer_.substr(0, newline);
      rx_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        lines.emplace_back(std::move(line));
      }
    }
    return true;
  }

private:
  int fd_{-1};
  std::string path_;
  std::string rx_buffer_;
};

struct BaseState
{
  bool seen{false};
  uint32_t state_sequence{0};
  uint32_t last_command_sequence{0};
  uint32_t uptime_ms{0};
  uint16_t status{0};
  uint16_t fault{0};
  uint32_t command_age_ms{0};
  std::array<double, 4> position{};
  std::array<double, 4> velocity{};
  uint32_t rx_errors{0};
  uint32_t last_valid_frame_age_ms{0};
  SteadyClock::time_point received_at{};
};

struct LiftState
{
  bool seen{false};
  uint32_t session{0};
  uint32_t last_sequence{0};
  std::string state{"WAIT_SESSION"};
  std::string fault{"NONE"};
  bool homed{false};
  bool armed{false};
  double position_m{0.0};
  double target_m{0.0};
  double velocity_m_s{0.0};
  bool lower_active{false};
  bool upper_active{false};
  SteadyClock::time_point received_at{};
};

struct ProtocolReply
{
  bool seen{false};
  bool ok{false};
  uint32_t sequence{0};
  std::string command;
  std::string reason;
  uint64_t generation{0};
};

bool validLiftStateName(const std::string & state)
{
  return state == "WAIT_SESSION" || state == "DISARMED" || state == "HOMING" ||
         state == "ARMED_IDLE" || state == "MOVING" || state == "FAULT";
}

bool validLiftFaultName(const std::string & fault)
{
  return fault == "NONE" || fault == "COMM_TIMEOUT" ||
         fault == "BOTH_LIMITS_ACTIVE" || fault == "HOMING_TIMEOUT" ||
         fault == "HOMING_RELEASE_FAILED" || fault == "ENDSTOP_POSITION_MISMATCH";
}

bool parseBoolParameter(const std::string & text, bool & value)
{
  if (text == "true" || text == "1") {
    value = true;
    return true;
  }
  if (text == "false" || text == "0") {
    value = false;
    return true;
  }
  return false;
}

}  // namespace

class TaiRobotSerialSystem::Impl
{
public:
  explicit Impl(TaiRobotSerialSystem & owner)
  : owner_(owner) {}

  bool initialise(const hardware_interface::HardwareInfo & info)
  {
    const auto parameter = [&info](const std::string & name, const std::string & fallback) {
        const auto found = info.hardware_parameters.find(name);
        return found == info.hardware_parameters.end() ? fallback : found->second;
      };

    drive_port_path_ = parameter("drive_serial_port", "/dev/tai_drive");
    lift_port_path_ = parameter("lift_serial_port", "/dev/tai_lift");
    if (!parseBoolParameter(parameter("use_lift", "true"), use_lift_)) {
      return failInit("use_lift must be true/false or 1/0");
    }
    if (!parseUnsignedParameter(parameter("drive_baud", "460800"), drive_baud_) ||
      drive_baud_ != 460800)
    {
      return failInit("drive_baud must be 460800 to match the base firmware");
    }
    if (!parseUnsignedParameter(parameter("lift_baud", "115200"), lift_baud_) ||
      lift_baud_ != 115200)
    {
      return failInit("lift_baud must be 115200 to match the lift firmware");
    }
    if (!parseUnsignedParameter(parameter("startup_timeout_ms", "3000"), startup_timeout_ms_) ||
      startup_timeout_ms_ < 500)
    {
      return failInit("startup_timeout_ms must be at least 500");
    }
    if (!parseUnsignedParameter(
        parameter("telemetry_timeout_ms", "250"), telemetry_timeout_ms_) ||
      telemetry_timeout_ms_ < 100)
    {
      return failInit("telemetry_timeout_ms must be at least 100");
    }
    if (!parseUnsignedParameter(
        parameter("lift_home_timeout_ms", "70000"), lift_home_timeout_ms_) ||
      lift_home_timeout_ms_ < 5000)
    {
      return failInit("lift_home_timeout_ms must be at least 5000");
    }
    if (!parseBoolParameter(
        parameter("home_lift_on_activate", "true"), home_lift_on_activate_))
    {
      return failInit("home_lift_on_activate must be true/false or 1/0");
    }
    if (!parsePositiveDouble(
        parameter("max_wheel_speed_rad_s", "5.7142857"), max_wheel_speed_rad_s_) ||
      max_wheel_speed_rad_s_ > kDefaultMaxWheelSpeedRadS + 1.0e-5)
    {
      return failInit("max_wheel_speed_rad_s must be positive and no greater than 5.7142857");
    }
    if (!parsePositiveDouble(
        parameter("max_lift_position_m", "0.520"), max_lift_position_m_) ||
      max_lift_position_m_ > kDefaultMaxLiftPositionM + 1.0e-6)
    {
      return failInit("max_lift_position_m must be positive and no greater than 0.520");
    }

    if (use_lift_ && drive_port_path_ == lift_port_path_) {
      return failInit("drive_serial_port and lift_serial_port must be different devices");
    }
    return validateJointContract(info);
  }

  hardware_interface::CallbackReturn configure()
  {
    resetRuntimeState();
    std::string error;
    if (!drive_port_.openPort(drive_port_path_, drive_baud_, error)) {
      RCLCPP_ERROR(owner_.get_logger(), "Base ESP32: %s", error.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (use_lift_ && !lift_port_.openPort(lift_port_path_, lift_baud_, error)) {
      RCLCPP_ERROR(owner_.get_logger(), "Lift ESP32: %s", error.c_str());
      drive_port_.closePort();
      return hardware_interface::CallbackReturn::ERROR;
    }

    setAllInterfacesToZero();
    lift_session_ = use_lift_ ? generateSessionId() : 0;

    // Opening either USB-UART may reset its ESP32. Retry only idempotent,
    // non-motion startup records while waiting for both firmwares to answer.
    if (!establishBaseSafeState() || (use_lift_ && !establishLiftSession())) {
      safeStopNoWait();
      closePorts();
      return hardware_interface::CallbackReturn::ERROR;
    }

    configured_ = true;
    if (use_lift_) {
      RCLCPP_INFO(
        owner_.get_logger(), "Connected base on %s and lift on %s; both are disarmed",
        drive_port_path_.c_str(), lift_port_path_.c_str());
    } else {
      RCLCPP_INFO(
        owner_.get_logger(),
        "Connected base on %s; combined IMU/lift bridge owns ESP32 number 2",
        drive_port_path_.c_str());
    }
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn activate()
  {
    if (!configured_) {
      RCLCPP_ERROR(owner_.get_logger(), "Cannot activate unconfigured serial hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }
    fatal_reason_.clear();

    // The base remains disabled throughout lift homing, so a long homing run
    // cannot expire the base motion watchdog.
    if (!prepareBaseForEnable() ||
      (use_lift_ && !prepareAndHomeLift()) ||
      !enableBase() || (use_lift_ && !armLift()))
    {
      safeStopNoWait();
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const char * joint : kWheelJointNames) {
      owner_.set_command(interfaceName(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
    }
    if (use_lift_) {
      owner_.set_command(
        interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION),
        lift_state_.position_m);
    }
    active_ = true;
    RCLCPP_INFO(
      owner_.get_logger(),
      "Serial hardware active: FL/FR/RL/RR capped at %.5f rad/s, lift at %.3f m",
      max_wheel_speed_rad_s_, use_lift_ ? lift_state_.position_m : 0.0);
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn deactivate()
  {
    active_ = false;
    safeStopNoWait();
    for (const char * joint : kWheelJointNames) {
      owner_.set_command(interfaceName(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
    }
    if (use_lift_ && lift_state_.seen) {
      owner_.set_command(
        interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION), lift_state_.position_m);
    }
    RCLCPP_INFO(owner_.get_logger(), "Serial hardware deactivated: base disabled, lift stopped");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn cleanup()
  {
    active_ = false;
    safeStopNoWait();
    closePorts();
    configured_ = false;
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type read()
  {
    if (!configured_) {
      return hardware_interface::return_type::ERROR;
    }
    if (!pumpBoth()) {
      return communicationError();
    }

    const auto now = SteadyClock::now();
    const auto timeout = std::chrono::milliseconds(telemetry_timeout_ms_);
    if (!base_state_.seen || now - base_state_.received_at > timeout) {
      setFatal("base telemetry timeout");
      return communicationError();
    }
    if (use_lift_ &&
      (!lift_state_.seen || now - lift_state_.received_at > timeout)) {
      setFatal("lift telemetry timeout");
      return communicationError();
    }
    if (!fatal_reason_.empty()) {
      return communicationError();
    }

    if (active_) {
      if (base_state_.fault != 0) {
        setFatal("base fault bits=" + std::to_string(base_state_.fault));
        return communicationError();
      }
      if ((base_state_.status & kBaseStatusEnabled) == 0U) {
        setFatal("base firmware reports drive disabled while hardware is active");
        return communicationError();
      }
      if (use_lift_ && lift_state_.fault != "NONE") {
        setFatal("lift fault=" + lift_state_.fault);
        return communicationError();
      }
      if (use_lift_ &&
        (!lift_state_.homed || !lift_state_.armed || lift_state_.session != lift_session_)) {
        setFatal("lift lost homed/armed session while hardware is active");
        return communicationError();
      }
    }

    for (std::size_t index = 0; index < kWheelJointNames.size(); ++index) {
      owner_.set_state(
        interfaceName(kWheelJointNames[index], hardware_interface::HW_IF_POSITION),
        base_state_.position[index]);
      owner_.set_state(
        interfaceName(kWheelJointNames[index], hardware_interface::HW_IF_VELOCITY),
        base_state_.velocity[index]);
    }
    if (use_lift_) {
      owner_.set_state(
        interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION),
        lift_state_.position_m);
      owner_.set_state(
        interfaceName(kLiftJointName, hardware_interface::HW_IF_VELOCITY),
        lift_state_.velocity_m_s);
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write()
  {
    if (!configured_ || !active_ || !fatal_reason_.empty()) {
      return hardware_interface::return_type::ERROR;
    }

    std::array<double, 4> wheel_commands{};
    for (std::size_t index = 0; index < kWheelJointNames.size(); ++index) {
      wheel_commands[index] = owner_.get_command<double>(
        interfaceName(kWheelJointNames[index], hardware_interface::HW_IF_VELOCITY));
      if (!std::isfinite(wheel_commands[index]) ||
        std::abs(wheel_commands[index]) > max_wheel_speed_rad_s_ + 1.0e-6)
      {
        setFatal("non-finite or out-of-range wheel command");
        safeStopNoWait();
        return hardware_interface::return_type::ERROR;
      }
    }
    double lift_command = 0.0;
    if (use_lift_) {
      lift_command = owner_.get_command<double>(
        interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION));
      if (!std::isfinite(lift_command) || lift_command < -1.0e-6 ||
        lift_command > max_lift_position_m_ + 1.0e-6)
      {
        setFatal("non-finite or out-of-range lift command");
        safeStopNoWait();
        return hardware_interface::return_type::ERROR;
      }
    }

    // Every update emits newly sequenced records. Nothing is queued for replay
    // after a disconnect; any write failure puts the component in ERROR.
    if (!sendBaseVelocity(wheel_commands) ||
      (use_lift_ &&
      !sendLiftSet(std::clamp(lift_command, 0.0, max_lift_position_m_))))
    {
      safeStopNoWait();
      return communicationError();
    }
    return hardware_interface::return_type::OK;
  }

private:
  bool failInit(const std::string & message)
  {
    RCLCPP_ERROR(owner_.get_logger(), "%s", message.c_str());
    return false;
  }

  static bool parseUnsignedParameter(const std::string & text, unsigned int & value)
  {
    uint32_t parsed = 0;
    if (!parseUint32(text, parsed) || parsed > std::numeric_limits<unsigned int>::max()) {
      return false;
    }
    value = static_cast<unsigned int>(parsed);
    return true;
  }

  static bool parsePositiveDouble(const std::string & text, double & value)
  {
    return parseDouble(text, value) && value > 0.0;
  }

  bool validateJointContract(const hardware_interface::HardwareInfo & info)
  {
    const std::size_t expected_joint_count = use_lift_ ? 5U : 4U;
    if (info.joints.size() != expected_joint_count) {
      return failInit(use_lift_ ?
        "TaiRobotSerialSystem requires exactly four wheel joints and lift_joint" :
        "TaiRobotSerialSystem requires exactly four wheel joints when use_lift=false");
    }
    std::unordered_map<std::string, const hardware_interface::ComponentInfo *> joints;
    for (const auto & joint : info.joints) {
      joints.emplace(joint.name, &joint);
    }
    for (const char * name : kWheelJointNames) {
      const auto found = joints.find(name);
      if (found == joints.end() || !validateJointInterfaces(
          *found->second, hardware_interface::HW_IF_VELOCITY))
      {
        return failInit(std::string("invalid ros2_control interfaces for ") + name);
      }
    }
    if (use_lift_) {
      const auto lift = joints.find(kLiftJointName);
      if (lift == joints.end() || !validateJointInterfaces(
          *lift->second, hardware_interface::HW_IF_POSITION))
      {
        return failInit("invalid ros2_control interfaces for lift_joint");
      }
    }
    return true;
  }

  static bool validateJointInterfaces(
    const hardware_interface::ComponentInfo & joint, const char * command_name)
  {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != command_name ||
      joint.state_interfaces.size() != 2)
    {
      return false;
    }
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & state : joint.state_interfaces) {
      has_position = has_position || state.name == hardware_interface::HW_IF_POSITION;
      has_velocity = has_velocity || state.name == hardware_interface::HW_IF_VELOCITY;
    }
    return has_position && has_velocity;
  }

  void setAllInterfacesToZero()
  {
    for (const char * joint : kWheelJointNames) {
      owner_.set_state(interfaceName(joint, hardware_interface::HW_IF_POSITION), 0.0);
      owner_.set_state(interfaceName(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
      owner_.set_command(interfaceName(joint, hardware_interface::HW_IF_VELOCITY), 0.0);
    }
    if (use_lift_) {
      owner_.set_state(interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION), 0.0);
      owner_.set_state(interfaceName(kLiftJointName, hardware_interface::HW_IF_VELOCITY), 0.0);
      owner_.set_command(interfaceName(kLiftJointName, hardware_interface::HW_IF_POSITION), 0.0);
    }
  }

  void resetRuntimeState()
  {
    configured_ = false;
    active_ = false;
    fatal_reason_.clear();
    base_state_ = BaseState{};
    lift_state_ = LiftState{};
    base_reply_ = ProtocolReply{};
    lift_reply_ = ProtocolReply{};
    base_reply_history_.clear();
    lift_reply_history_.clear();
    base_reply_generation_ = 0;
    lift_reply_generation_ = 0;
    base_sequence_ = 0;
    lift_sequence_ = 0;
    base_hello_seen_ = false;
    lift_hello_ack_seen_ = false;
    lift_home_complete_seen_ = false;
  }

  uint32_t generateSessionId()
  {
    std::random_device random;
    const auto ticks = static_cast<uint64_t>(SteadyClock::now().time_since_epoch().count());
    uint32_t session = static_cast<uint32_t>(ticks) ^
      static_cast<uint32_t>(ticks >> 32) ^ random() ^ static_cast<uint32_t>(::getpid());
    if (session == 0 || session == previous_lift_session_) {
      ++session;
    }
    previous_lift_session_ = session;
    return session;
  }

  void closePorts()
  {
    drive_port_.closePort();
    lift_port_.closePort();
  }

  void setFatal(const std::string & reason)
  {
    if (fatal_reason_.empty()) {
      fatal_reason_ = reason;
      RCLCPP_ERROR(owner_.get_logger(), "Serial hardware fault: %s", reason.c_str());
    }
  }

  hardware_interface::return_type communicationError()
  {
    active_ = false;
    safeStopNoWait();
    if (fatal_reason_.empty()) {
      fatal_reason_ = "unspecified communication error";
    }
    return hardware_interface::return_type::ERROR;
  }

  bool pumpBoth()
  {
    return pumpBase() && (!use_lift_ || pumpLift());
  }

  bool pumpBase()
  {
    std::vector<std::string> lines;
    std::string error;
    if (!drive_port_.readLines(lines, error)) {
      setFatal("base serial: " + error);
      return false;
    }
    for (const auto & line : lines) {
      if (!processBaseLine(line)) {
        return false;
      }
    }
    return true;
  }

  bool pumpLift()
  {
    std::vector<std::string> lines;
    std::string error;
    if (!lift_port_.readLines(lines, error)) {
      setFatal("lift serial: " + error);
      return false;
    }
    for (const auto & line : lines) {
      if (!processLiftLine(line)) {
        return false;
      }
    }
    return true;
  }

  bool processBaseLine(const std::string & line)
  {
    std::string payload;
    if (!decodeBaseFrame(line, payload)) {
      setFatal("base sent a malformed or CRC-invalid frame");
      return false;
    }
    const auto fields = split(payload, ',');
    if (fields.empty()) {
      setFatal("base sent an empty frame");
      return false;
    }

    if (fields[0] == "HELLO") {
      uint16_t version = 0;
      double reported_limit = 0.0;
      if (fields.size() != 7 || !parseUint16(fields[1], version) ||
        version != kBaseProtocolVersion || fields[2] != "BASE_DRIVE" ||
        fields[3] != "FL_FR_RL_RR" || !parseDouble(fields[4], reported_limit) ||
        std::abs(reported_limit - kDefaultMaxWheelSpeedRadS) > 1.0e-3 ||
        fields[5] != "300" || fields[6] != "1000")
      {
        setFatal("base HELLO does not match protocol v1");
        return false;
      }
      if (active_) {
        setFatal("base ESP32 restarted while hardware was active");
        return false;
      }
      base_hello_seen_ = true;
      return true;
    }

    if (fields[0] == "ACK" || fields[0] == "NACK") {
      uint16_t version = 0;
      uint32_t sequence = 0;
      if (fields.size() != 5 || !parseUint16(fields[1], version) ||
        version != kBaseProtocolVersion || !parseUint32(fields[2], sequence))
      {
        setFatal("base sent a malformed ACK/NACK");
        return false;
      }
      base_reply_.seen = true;
      base_reply_.ok = fields[0] == "ACK" && fields[3] == "OK";
      base_reply_.sequence = sequence;
      base_reply_.reason = fields[3];
      base_reply_.command = fields[4];
      base_reply_.generation = ++base_reply_generation_;
      base_reply_history_.push_back(base_reply_);
      if (base_reply_history_.size() > kReplyHistoryLimit) {
        base_reply_history_.pop_front();
      }
      if (!base_reply_.ok && active_) {
        setFatal("base rejected " + fields[4] + ": " + fields[3]);
        return false;
      }
      return true;
    }

    if (fields[0] != "STATE" || fields.size() != 26) {
      setFatal("base sent an unknown or malformed record");
      return false;
    }

    BaseState parsed;
    uint16_t version = 0;
    if (!parseUint16(fields[1], version) || version != kBaseProtocolVersion ||
      !parseUint32(fields[2], parsed.state_sequence) ||
      !parseUint32(fields[3], parsed.last_command_sequence) ||
      !parseUint32(fields[4], parsed.uptime_ms) ||
      !parseUint16(fields[5], parsed.status) || !parseUint16(fields[6], parsed.fault) ||
      !parseUint32(fields[7], parsed.command_age_ms))
    {
      setFatal("base STATE header is malformed");
      return false;
    }
    for (std::size_t index = 0; index < 4; ++index) {
      if (!parseDouble(fields[8 + index], parsed.position[index]) ||
        !parseDouble(fields[12 + index], parsed.velocity[index]))
      {
        setFatal("base STATE wheel feedback is malformed");
        return false;
      }
      double ignored_target = 0.0;
      if (!parseDouble(fields[16 + index], ignored_target)) {
        setFatal("base STATE target is malformed");
        return false;
      }
      errno = 0;
      char * end = nullptr;
      const int64_t parsed_pwm = std::strtoll(fields[20 + index].c_str(), &end, 10);
      if (errno != 0 || end == fields[20 + index].c_str() || *end != '\0' ||
        parsed_pwm < -255 || parsed_pwm > 255)
      {
        setFatal("base STATE PWM is malformed");
        return false;
      }
    }
    if (!parseUint32(fields[24], parsed.rx_errors) ||
      !parseUint32(fields[25], parsed.last_valid_frame_age_ms))
    {
      setFatal("base STATE diagnostics are malformed");
      return false;
    }
    if (base_state_.seen && !sequenceNewer(parsed.state_sequence, base_state_.state_sequence)) {
      setFatal("base sent stale or replayed telemetry");
      return false;
    }
    if (base_state_.seen && parsed.uptime_ms != base_state_.uptime_ms &&
      !sequenceNewer(parsed.uptime_ms, base_state_.uptime_ms))
    {
      setFatal("base ESP32 uptime moved backwards (reset detected)");
      return false;
    }
    if (active_ && base_state_.seen && parsed.rx_errors > base_state_.rx_errors) {
      setFatal("base firmware reports a newly rejected host frame");
      return false;
    }
    parsed.seen = true;
    parsed.received_at = SteadyClock::now();
    base_state_ = parsed;
    return true;
  }

  bool processLiftLine(const std::string & line)
  {
    const auto fields = split(line, ',');
    if (fields.empty()) {
      setFatal("lift sent an empty record");
      return false;
    }
    if (fields[0] == "BOOT") {
      if (fields.size() != 3 || fields[1] != "LIFT_CONTROL" || fields[2] != "1") {
        setFatal("lift BOOT does not match protocol v1");
        return false;
      }
      if (active_) {
        setFatal("lift ESP32 restarted while hardware was active");
        return false;
      }
      return true;
    }
    if (fields[0] == "ACK") {
      uint32_t session = 0;
      uint32_t sequence = 0;
      if (fields.size() != 4 || !parseUint32(fields[1], session) ||
        !parseUint32(fields[2], sequence) || session != lift_session_)
      {
        setFatal("lift sent a malformed or wrong-session ACK");
        return false;
      }
      lift_reply_.seen = true;
      lift_reply_.ok = true;
      lift_reply_.sequence = sequence;
      lift_reply_.command = fields[3];
      lift_reply_.reason.clear();
      lift_reply_.generation = ++lift_reply_generation_;
      lift_reply_history_.push_back(lift_reply_);
      if (lift_reply_history_.size() > kReplyHistoryLimit) {
        lift_reply_history_.pop_front();
      }
      if (fields[3] == "HELLO" && sequence == 0) {
        lift_hello_ack_seen_ = true;
      }
      return true;
    }
    if (fields[0] == "ERR") {
      uint32_t session = 0;
      uint32_t sequence = 0;
      if (fields.size() != 4 || !parseUint32(fields[1], session) ||
        !parseUint32(fields[2], sequence))
      {
        setFatal("lift sent a malformed ERR");
        return false;
      }
      lift_reply_.seen = true;
      lift_reply_.ok = false;
      lift_reply_.sequence = sequence;
      lift_reply_.command.clear();
      lift_reply_.reason = fields[3];
      lift_reply_.generation = ++lift_reply_generation_;
      lift_reply_history_.push_back(lift_reply_);
      if (lift_reply_history_.size() > kReplyHistoryLimit) {
        lift_reply_history_.pop_front();
      }
      if (active_) {
        setFatal("lift rejected command: " + fields[3]);
        return false;
      }
      return true;
    }
    if (fields[0] == "EVENT") {
      if (fields.size() != 2 ||
        (fields[1] != "HOME_STARTED" && fields[1] != "HOME_RELEASE_LOWER" &&
        fields[1] != "HOME_RELEASE_UPPER" && fields[1] != "HOME_COMPLETE" &&
        fields[1] != "COMM_TIMEOUT" &&
        fields[1] != "HOMING_TIMEOUT" && fields[1] != "HOMING_RELEASE_FAILED"))
      {
        setFatal("lift sent an unknown EVENT");
        return false;
      }
      if (fields[1] == "HOME_COMPLETE") {
        lift_home_complete_seen_ = true;
        return true;
      }
      if (fields[1] == "COMM_TIMEOUT" || fields[1] == "HOMING_TIMEOUT" ||
        fields[1] == "HOMING_RELEASE_FAILED")
      {
        setFatal("lift event=" + fields[1]);
        return false;
      }
      return true;
    }
    if (fields[0] != "STATE" || fields.size() != 12) {
      setFatal("lift sent an unknown or malformed record");
      return false;
    }

    LiftState parsed;
    if (!parseUint32(fields[1], parsed.session) ||
      !parseUint32(fields[2], parsed.last_sequence) ||
      !validLiftStateName(fields[3]) || !validLiftFaultName(fields[4]) ||
      !parseFlag(fields[5], parsed.homed) || !parseFlag(fields[6], parsed.armed) ||
      !parseDouble(fields[7], parsed.position_m) ||
      !parseDouble(fields[8], parsed.target_m) ||
      !parseDouble(fields[9], parsed.velocity_m_s) ||
      !parseFlag(fields[10], parsed.lower_active) || !parseFlag(fields[11], parsed.upper_active))
    {
      setFatal("lift STATE is malformed");
      return false;
    }
    parsed.state = fields[3];
    parsed.fault = fields[4];
    if (parsed.session != lift_session_) {
      // A boot-time STATE,0 line may precede the HELLO ACK in the same USB
      // buffer. Once HELLO is acknowledged, a session mismatch is a fault.
      if (parsed.session == 0 && !lift_hello_ack_seen_) {
        return true;
      }
      setFatal("lift STATE belongs to a stale session");
      return false;
    }
    if (lift_state_.seen && parsed.last_sequence != lift_state_.last_sequence &&
      !sequenceNewer(parsed.last_sequence, lift_state_.last_sequence))
    {
      setFatal("lift sent stale or replayed telemetry");
      return false;
    }
    if (parsed.position_m < -0.02 || parsed.position_m > kDefaultMaxLiftPositionM + 0.02 ||
      parsed.target_m < -0.02 || parsed.target_m > kDefaultMaxLiftPositionM + 0.02 ||
      std::abs(parsed.velocity_m_s) > 0.06)
    {
      setFatal("lift STATE contains physically invalid values");
      return false;
    }
    parsed.seen = true;
    parsed.received_at = SteadyClock::now();
    lift_state_ = parsed;
    return true;
  }

  bool sendBasePayload(const std::string & payload)
  {
    std::string error;
    if (!drive_port_.writeLine(frameBasePayload(payload), error)) {
      setFatal("base serial: " + error);
      return false;
    }
    return true;
  }

  uint32_t nextBaseSequence() {return ++base_sequence_;}
  uint32_t nextLiftSequence() {return ++lift_sequence_;}

  bool sendBaseSimple(const std::string & command, uint32_t & sequence)
  {
    sequence = nextBaseSequence();
    return sendBasePayload(
      command + ",1," + std::to_string(sequence));
  }

  bool sendBaseVelocity(const std::array<double, 4> & command, uint32_t * sent_sequence = nullptr)
  {
    const uint32_t sequence = nextBaseSequence();
    std::ostringstream payload;
    payload.imbue(std::locale::classic());
    payload << "CMD,1," << sequence << std::fixed << std::setprecision(6);
    for (const double velocity : command) {
      payload << ',' << velocity;
    }
    if (!sendBasePayload(payload.str())) {
      return false;
    }
    if (sent_sequence != nullptr) {
      *sent_sequence = sequence;
    }
    return true;
  }

  bool sendLiftLine(const std::string & line)
  {
    std::string error;
    if (!lift_port_.writeLine(line + "\n", error)) {
      setFatal("lift serial: " + error);
      return false;
    }
    return true;
  }

  bool sendLiftSimple(const std::string & command, uint32_t & sequence)
  {
    sequence = nextLiftSequence();
    return sendLiftLine(
      command + " " + std::to_string(lift_session_) + " " + std::to_string(sequence));
  }

  bool sendLiftSet(double position_m, uint32_t * sent_sequence = nullptr)
  {
    const uint32_t sequence = nextLiftSequence();
    std::ostringstream line;
    line.imbue(std::locale::classic());
    line << "SET " << lift_session_ << ' ' << sequence << ' ' << std::fixed <<
      std::setprecision(5) << position_m;
    if (!sendLiftLine(line.str())) {
      return false;
    }
    if (sent_sequence != nullptr) {
      *sent_sequence = sequence;
    }
    return true;
  }

  bool waitBaseReply(
    uint32_t sequence, const std::string & command, unsigned int timeout_ms,
    bool keep_lift_alive = false)
  {
    const uint64_t initial_generation = base_reply_generation_;
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    auto next_lift_heartbeat = SteadyClock::now();
    while (SteadyClock::now() < deadline) {
      if (!pumpBoth()) {
        return false;
      }
      const auto reply = std::find_if(
        base_reply_history_.begin(), base_reply_history_.end(),
        [initial_generation, sequence, &command](const ProtocolReply & candidate) {
          return candidate.generation > initial_generation && candidate.sequence == sequence &&
                 candidate.command == command;
        });
      if (reply != base_reply_history_.end()) {
        if (!reply->ok) {
          RCLCPP_ERROR(
            owner_.get_logger(), "Base rejected %s: %s", command.c_str(),
            reply->reason.c_str());
        }
        return reply->ok;
      }
      if (keep_lift_alive && SteadyClock::now() >= next_lift_heartbeat) {
        uint32_t ignored = 0;
        if (!sendLiftSimple("HB", ignored)) {
          return false;
        }
        next_lift_heartbeat = SteadyClock::now() + std::chrono::milliseconds(100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_ERROR(owner_.get_logger(), "Timed out waiting for base %s ACK", command.c_str());
    return false;
  }

  bool waitLiftReply(
    uint32_t sequence, const std::string & command, unsigned int timeout_ms,
    bool keep_base_alive = false)
  {
    const uint64_t initial_generation = lift_reply_generation_;
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    auto next_base_command = SteadyClock::now();
    while (SteadyClock::now() < deadline) {
      if (!pumpBoth()) {
        return false;
      }
      const auto reply = std::find_if(
        lift_reply_history_.begin(), lift_reply_history_.end(),
        [initial_generation, sequence](const ProtocolReply & candidate) {
          return candidate.generation > initial_generation && candidate.sequence == sequence;
        });
      if (reply != lift_reply_history_.end()) {
        if (!reply->ok || reply->command != command) {
          RCLCPP_ERROR(
            owner_.get_logger(), "Lift rejected %s: %s", command.c_str(),
            reply->reason.c_str());
          return false;
        }
        return true;
      }
      if (keep_base_alive && SteadyClock::now() >= next_base_command) {
        if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0})) {
          return false;
        }
        next_base_command = SteadyClock::now() + std::chrono::milliseconds(100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_ERROR(owner_.get_logger(), "Timed out waiting for lift %s ACK", command.c_str());
    return false;
  }

  bool waitForState(
    const std::function<bool()> & predicate, unsigned int timeout_ms,
    bool keep_base_alive, bool keep_lift_alive,
    const std::function<bool()> & failure_predicate = {})
  {
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(timeout_ms);
    auto next_base_command = SteadyClock::now();
    auto next_lift_heartbeat = SteadyClock::now();
    while (SteadyClock::now() < deadline) {
      if (!pumpBoth()) {
        return false;
      }
      if (predicate()) {
        return true;
      }
      if (failure_predicate && failure_predicate()) {
        return false;
      }
      const auto now = SteadyClock::now();
      if (keep_base_alive && now >= next_base_command) {
        if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0})) {
          return false;
        }
        next_base_command = now + std::chrono::milliseconds(100);
      }
      if (keep_lift_alive && now >= next_lift_heartbeat) {
        uint32_t ignored = 0;
        if (!sendLiftSimple("HB", ignored)) {
          return false;
        }
        next_lift_heartbeat = now + std::chrono::milliseconds(100);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return false;
  }

  bool establishBaseSafeState()
  {
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(startup_timeout_ms_);
    while (SteadyClock::now() < deadline) {
      fatal_reason_.clear();
      uint32_t sequence = 0;
      if (sendBaseSimple("DISABLE", sequence) &&
        waitBaseReply(sequence, "DISABLE", 300))
      {
        if (waitForState([this]() {return base_state_.seen;}, 500, false, false)) {
          return true;
        }
      }
      if (!drive_port_.isOpen()) {
        break;
      }
      // A USB reset can yield a partial boot line; retry after clearing only
      // startup parser state, never after the component has become active.
      fatal_reason_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_ERROR(owner_.get_logger(), "Base ESP32 did not enter DISABLED state");
    return false;
  }

  bool establishLiftSession()
  {
    const auto deadline = SteadyClock::now() + std::chrono::milliseconds(startup_timeout_ms_);
    while (SteadyClock::now() < deadline) {
      fatal_reason_.clear();
      const uint64_t initial_generation = lift_reply_generation_;
      if (!sendLiftLine("HELLO " + std::to_string(lift_session_))) {
        return false;
      }
      const auto attempt_deadline = SteadyClock::now() + std::chrono::milliseconds(400);
      while (SteadyClock::now() < attempt_deadline) {
        if (!pumpBoth()) {
          break;
        }
        const auto reply = std::find_if(
          lift_reply_history_.begin(), lift_reply_history_.end(),
          [initial_generation](const ProtocolReply & candidate) {
            return candidate.generation > initial_generation && candidate.ok &&
                   candidate.sequence == 0 && candidate.command == "HELLO";
          });
        if (reply != lift_reply_history_.end()) {
          if (waitForState(
              [this]() {return lift_state_.seen && lift_state_.session == lift_session_;},
              500, false, false))
          {
            return true;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      fatal_reason_.clear();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_ERROR(owner_.get_logger(), "Lift ESP32 did not accept a new session");
    return false;
  }

  bool prepareBaseForEnable()
  {
    uint32_t sequence = 0;
    if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0}, &sequence) ||
      !waitBaseReply(sequence, "CMD", startup_timeout_ms_))
    {
      return false;
    }
    if (!waitForState([this]() {return base_state_.seen;}, 500, false, false)) {
      return false;
    }
    if ((base_state_.fault & kBaseFaultSoftwareEstop) != 0U) {
      RCLCPP_ERROR(
        owner_.get_logger(),
        "Base software E-stop is latched; inspect the robot and clear it explicitly before launch");
      return false;
    }
    if (base_state_.fault != 0U && base_state_.fault != kBaseFaultCommandTimeout) {
      RCLCPP_ERROR(
        owner_.get_logger(), "Refusing to auto-clear unknown base fault bits=%u",
          base_state_.fault);
      return false;
    }
    if (base_state_.fault == kBaseFaultCommandTimeout) {
      // CLEAR itself does not renew the motion lease. Refresh the required
      // all-zero CMD immediately before asking the MCU to clear the latch.
      if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0}, &sequence) ||
        !waitBaseReply(sequence, "CMD", startup_timeout_ms_) ||
        !sendBaseSimple("CLEAR", sequence) ||
        !waitBaseReply(sequence, "CLEAR", startup_timeout_ms_))
      {
        return false;
      }
      if (!waitForState(
          [this]() {return base_state_.seen && base_state_.fault == 0U;}, 500, false, false))
      {
        RCLCPP_ERROR(owner_.get_logger(), "Base command-timeout fault did not clear");
        return false;
      }
    }
    return true;
  }

  bool prepareAndHomeLift()
  {
    if (lift_state_.fault != "NONE") {
      if (lift_state_.fault != "COMM_TIMEOUT") {
        RCLCPP_ERROR(
          owner_.get_logger(), "Refusing to auto-clear lift fault: %s",
          lift_state_.fault.c_str());
        return false;
      }
      uint32_t sequence = 0;
      if (!sendLiftSimple("CLEAR", sequence) ||
        !waitLiftReply(sequence, "CLEAR", startup_timeout_ms_))
      {
        return false;
      }
      if (!waitForState(
          [this]() {return lift_state_.seen && lift_state_.fault == "NONE";},
          500, false, false))
      {
        return false;
      }
    }

    if (lift_state_.homed) {
      return true;
    }
    if (!home_lift_on_activate_) {
      RCLCPP_ERROR(
        owner_.get_logger(), "Lift is not homed and home_lift_on_activate is false");
      return false;
    }

    RCLCPP_WARN(
      owner_.get_logger(),
      "Homing lift now: forks may first move upward up to 0.025 m, then seek the lower switch");
    uint32_t sequence = 0;
    lift_home_complete_seen_ = false;
    if (!sendLiftSimple("HOME", sequence) ||
      !waitLiftReply(sequence, "HOME", startup_timeout_ms_))
    {
      return false;
    }
    const bool homed = waitForState(
      [this]() {
        return lift_state_.seen && lift_state_.fault == "NONE" && lift_state_.homed &&
               !lift_state_.armed && (lift_home_complete_seen_ || lift_state_.state == "DISARMED");
      },
      lift_home_timeout_ms_, false, true,
      [this]() {return lift_state_.seen && lift_state_.fault != "NONE";});
    if (!homed) {
      RCLCPP_ERROR(owner_.get_logger(), "Lift homing failed or timed out");
    }
    return homed;
  }

  bool enableBase()
  {
    uint32_t sequence = 0;
    // Lift homing can take 65 seconds, so the zero command used before homing
    // is no longer fresh enough for ENABLE. Renew it here first.
    if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0}, &sequence) ||
      !waitBaseReply(sequence, "CMD", startup_timeout_ms_) ||
      !sendBaseSimple("ENABLE", sequence) ||
      !waitBaseReply(sequence, "ENABLE", startup_timeout_ms_))
    {
      return false;
    }
    if (!sendBaseVelocity({0.0, 0.0, 0.0, 0.0}, &sequence) ||
      !waitBaseReply(sequence, "CMD", startup_timeout_ms_))
    {
      return false;
    }
    return waitForState(
      [this]() {
        return base_state_.seen && base_state_.fault == 0U &&
               (base_state_.status & kBaseStatusEnabled) != 0U;
      },
      500, true, false);
  }

  bool armLift()
  {
    uint32_t sequence = 0;
    if (!sendLiftSimple("ARM", sequence) ||
      !waitLiftReply(sequence, "ARM", startup_timeout_ms_, true))
    {
      return false;
    }
    if (!sendLiftSet(lift_state_.position_m, &sequence) ||
      !waitLiftReply(sequence, "SET", startup_timeout_ms_, true))
    {
      return false;
    }
    return waitForState(
      [this]() {
        return lift_state_.seen && lift_state_.fault == "NONE" && lift_state_.homed &&
               lift_state_.armed;
      },
      500, true, true);
  }

  void safeStopNoWait()
  {
    if (drive_port_.isOpen()) {
      (void)sendBaseVelocity({0.0, 0.0, 0.0, 0.0});
      uint32_t ignored = 0;
      (void)sendBaseSimple("DISABLE", ignored);
    }
    if (lift_port_.isOpen() && lift_session_ != 0) {
      uint32_t ignored = 0;
      (void)sendLiftSimple("STOP", ignored);
    }
  }

  TaiRobotSerialSystem & owner_;
  SerialPort drive_port_;
  SerialPort lift_port_;
  std::string drive_port_path_{"/dev/tai_drive"};
  std::string lift_port_path_{"/dev/tai_lift"};
  unsigned int drive_baud_{460800};
  unsigned int lift_baud_{115200};
  unsigned int startup_timeout_ms_{3000};
  unsigned int telemetry_timeout_ms_{250};
  unsigned int lift_home_timeout_ms_{70000};
  bool home_lift_on_activate_{true};
  bool use_lift_{true};
  double max_wheel_speed_rad_s_{kDefaultMaxWheelSpeedRadS};
  double max_lift_position_m_{kDefaultMaxLiftPositionM};

  bool configured_{false};
  bool active_{false};
  bool base_hello_seen_{false};
  bool lift_hello_ack_seen_{false};
  bool lift_home_complete_seen_{false};
  std::string fatal_reason_;

  uint32_t base_sequence_{0};
  uint32_t lift_sequence_{0};
  uint32_t lift_session_{0};
  uint32_t previous_lift_session_{0};
  BaseState base_state_;
  LiftState lift_state_;
  ProtocolReply base_reply_;
  ProtocolReply lift_reply_;
  static constexpr std::size_t kReplyHistoryLimit = 512;
  std::deque<ProtocolReply> base_reply_history_;
  std::deque<ProtocolReply> lift_reply_history_;
  uint64_t base_reply_generation_{0};
  uint64_t lift_reply_generation_{0};
};

TaiRobotSerialSystem::TaiRobotSerialSystem()
: impl_(std::make_unique<Impl>(*this)) {}

TaiRobotSerialSystem::~TaiRobotSerialSystem() = default;

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return impl_->initialise(info_) ? hardware_interface::CallbackReturn::SUCCESS :
         hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->configure();
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->cleanup();
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->activate();
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->deactivate();
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->cleanup();
}

hardware_interface::CallbackReturn TaiRobotSerialSystem::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return impl_->cleanup();
}

hardware_interface::return_type TaiRobotSerialSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return impl_->read();
}

hardware_interface::return_type TaiRobotSerialSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return impl_->write();
}

}  // namespace tai_robot_one

PLUGINLIB_EXPORT_CLASS(
  tai_robot_one::TaiRobotSerialSystem, hardware_interface::SystemInterface)
