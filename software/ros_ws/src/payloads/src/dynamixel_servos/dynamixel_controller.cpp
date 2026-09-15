#include "dynamixel_controller.hpp"

#include <asm/termbits.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace payloads {
namespace {

// FTDI adapters buffer received bytes until latency_timer ms elapse, 16 ms by
// default, which alone caps a request/response protocol near 30 Hz. ftdi_sio
// maps ASYNC_LOW_LATENCY onto that register and drops it to 1 ms. The flag is
// in ASYNC_USR_MASK, so setting it needs no CAP_SYS_ADMIN.
//
// The value is per-port driver state rather than per-fd, so a temporary fd
// suffices; PortHandlerLinux does not expose its own. Drivers other than
// ftdi_sio implement no set_serial op and fail with ENOTTY.
//
// Returns an error description, or empty on success.
std::string applyLowLatency(const std::string &device) {
  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return std::string("open: ") + std::strerror(errno);
  }

  std::string error;
  struct serial_struct serial_info{};
  if (::ioctl(fd, TIOCGSERIAL, &serial_info) != 0) {
    error = std::string("TIOCGSERIAL: ") + std::strerror(errno);
  } else {
    serial_info.flags |= ASYNC_LOW_LATENCY;
    if (::ioctl(fd, TIOCSSERIAL, &serial_info) != 0) {
      error = std::string("TIOCSSERIAL: ") + std::strerror(errno);
    }
  }

  ::close(fd);
  return error;
}

std::string applyBaudRate(const std::string &device, int baud_rate) {
  int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    return std::string("open: ") + std::strerror(errno);
  }

  std::string error;
  struct termios2 to{};
  if (::ioctl(fd, TCGETS2, &to) != 0) {
    error = std::string("TCGETS2: ") + std::strerror(errno);
  } else {
    to.c_cflag &= ~(CBAUD | CIBAUD | CSIZE | CSTOPB | PARENB);
    to.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
    to.c_ispeed = baud_rate;
    to.c_ospeed = baud_rate;
    if (::ioctl(fd, TCSETS2, &to) != 0) {
      error = std::string("TCSETS2: ") + std::strerror(errno);
    }
  }

  ::close(fd);
  return error;
}

} // namespace

double DynamixelController::unitsToRadians(int32_t units) {
  constexpr double tau = 6.28318530717958647692;
  return static_cast<double>(units) * tau / (counts_per_turn * gear_ratio);
}

int32_t DynamixelController::radiansToUnits(double radians) {
  constexpr double tau = 6.28318530717958647692;
  const double units = radians * counts_per_turn * gear_ratio / tau;
  if (!std::isfinite(units) || units < std::numeric_limits<int32_t>::min() ||
      units > std::numeric_limits<int32_t>::max()) {
    throw std::out_of_range(
        "Radian target is outside the 32-bit position range");
  }
  return static_cast<int32_t>(std::llround(units));
}

DynamixelController::DynamixelController(const std::string &device,
                                         int baud_rate)
    : connector_(device, baud_rate) {
  if (const auto err = applyBaudRate(device, baud_rate); !err.empty()) {
    throw std::runtime_error("Could not set " + std::to_string(baud_rate) +
                             " baud on " + device + ": " + err);
  }
  low_latency_error_ = applyLowLatency(device);
}

DynamixelController::~DynamixelController() noexcept {
  for (auto &[id, motor] : servos_) {
    (void)id;
    (void)motor->disableTorque();
  }
}

dynamixel::Result<std::vector<uint8_t>, dynamixel::DxlError>
DynamixelController::scan() {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto result = connector_.broadcastPing();
  if (!result.isSuccess()) {
    return result.error();
  }
  for (uint8_t id : result.value()) {
    if (servos_.find(id) == servos_.end()) {
      servos_.emplace(id, connector_.createMotor(id));
    }
  }
  return result.value();
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::enableTorque(uint8_t id) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return servo(id).enableTorque();
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::disableTorque(uint8_t id) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return servo(id).disableTorque();
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::setMode(uint8_t id, Mode mode) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  // Operating mode can only be changed when torque is disabled.
  auto torque_off = servo(id).disableTorque();
  if (!torque_off.isSuccess()) {
    return torque_off.error();
  }
  return servo(id).setOperatingMode(mode);
}

dynamixel::Result<DynamixelController::Mode, dynamixel::DxlError>
DynamixelController::mode(uint8_t id) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return servo(id).getOperatingMode();
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::setTargetPosition(uint8_t id, double radians) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return servo(id).setGoalPosition(radiansToUnits(radians));
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::setTargetPosition(
    const std::unordered_map<uint8_t, double> &radians) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  return setTargetPositionsLocked(radians);
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::setTargetPosition(double radians) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  std::unordered_map<uint8_t, double> targets;
  targets.reserve(servos_.size());
  for (const auto &[id, motor] : servos_) {
    (void)motor;
    targets.emplace(id, radians);
  }
  return setTargetPositionsLocked(targets);
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::setTargetPositionsLocked(
    const std::unordered_map<uint8_t, double> &radians) {
  if (radians.empty()) {
    return {};
  }
  auto executor = connector_.createGroupExecutor();
  for (const auto &[id, target] : radians) {
    auto command = servo(id).stageSetGoalPosition(radiansToUnits(target));
    if (!command.isSuccess()) {
      return command.error();
    }
    executor->addCmd(command.value());
  }
  return executor->executeWrite();
}

dynamixel::Result<void, dynamixel::DxlError>
DynamixelController::moveToMiddle(uint8_t id) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto minimum = servo(id).getMinPositionLimit();
  if (!minimum.isSuccess()) {
    return minimum.error();
  }

  auto maximum = servo(id).getMaxPositionLimit();
  if (!maximum.isSuccess()) {
    return maximum.error();
  }

  const auto middle = static_cast<int32_t>(
      minimum.value() + ((maximum.value() - minimum.value()) / 2));
  return servo(id).setGoalPosition(middle);
}

dynamixel::Result<double, dynamixel::DxlError>
DynamixelController::readPosition(uint8_t id) {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  auto result = servo(id).getPresentPosition();
  if (!result.isSuccess()) {
    return result.error();
  }

  const double radians = unitsToRadians(result.value());
  {
    std::lock_guard<std::mutex> positions_lock(positions_mutex_);
    positions_[id] = radians;
  }
  return radians;
}

dynamixel::Result<std::unordered_map<uint8_t, double>, dynamixel::DxlError>
DynamixelController::readPositions() {
  std::lock_guard<std::mutex> lock(bus_mutex_);
  // Nothing on the bus (every servo mocked, or scan() found none): a group
  // executor with no staged commands has no defined result, so short-circuit.
  if (servos_.empty()) {
    return std::unordered_map<uint8_t, double>{};
  }

  auto executor = connector_.createGroupExecutor();
  std::vector<uint8_t> ids;
  ids.reserve(servos_.size());

  for (auto &[id, motor] : servos_) {
    auto command = motor->stageGetPresentPosition();
    if (!command.isSuccess()) {
      return command.error();
    }
    ids.push_back(id);
    executor->addCmd(command.value());
  }

  auto values = executor->executeRead();
  if (!values.isSuccess()) {
    return values.error();
  }

  std::unordered_map<uint8_t, double> positions;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    auto &value = values.value()[index];
    if (!value.isSuccess()) {
      return value.error();
    }
    positions.emplace(ids[index], unitsToRadians(value.value()));
  }

  {
    std::lock_guard<std::mutex> positions_lock(positions_mutex_);
    positions_ = positions;
  }
  return positions;
}

std::unordered_map<uint8_t, double> DynamixelController::memPositions() const {
  std::lock_guard<std::mutex> lock(positions_mutex_);
  return positions_;
}

dynamixel::Motor &DynamixelController::servo(uint8_t id) {
  auto found = servos_.find(id);
  if (found == servos_.end()) {
    throw std::out_of_range("Servo ID " + std::to_string(id) +
                            " was not found by scan()");
  }
  return *found->second;
}

} // namespace payloads
