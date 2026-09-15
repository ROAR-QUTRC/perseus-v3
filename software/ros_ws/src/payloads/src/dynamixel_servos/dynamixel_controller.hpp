#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <dynamixel_easy_sdk/connector.hpp>

namespace payloads {

/// Thin, thread-safe wrapper over the Dynamixel easy-SDK.
///
/// Owns the bus connection and one Motor per servo discovered by scan(), and
/// exposes group (sync) reads and writes so every servo is serviced in a single
/// bus round trip.
class DynamixelController {
public:
  using Mode = dynamixel::OperatingMode;
  // modes:
  // CURRENT, VELOCITY, POSITION, EXTENDED_POSITION, CURRENT_BASED_POSITION, PWM

  static constexpr float protocol_version = 2.0F;
  static constexpr int32_t counts_per_turn = 4096;
  static constexpr double gear_ratio = 1.0; // Motor turns per output turn.

  static double unitsToRadians(int32_t units);
  static int32_t radiansToUnits(double radians);

  DynamixelController(const std::string &device, int baud_rate);
  ~DynamixelController() noexcept;

  dynamixel::Result<std::vector<uint8_t>, dynamixel::DxlError> scan();

  dynamixel::Result<void, dynamixel::DxlError> enableTorque(uint8_t id);
  dynamixel::Result<void, dynamixel::DxlError> disableTorque(uint8_t id);
  dynamixel::Result<void, dynamixel::DxlError> setMode(uint8_t id, Mode mode);
  dynamixel::Result<Mode, dynamixel::DxlError> mode(uint8_t id);
  dynamixel::Result<void, dynamixel::DxlError>
  setTargetPosition(uint8_t id, double radians);
  dynamixel::Result<void, dynamixel::DxlError>
  setTargetPosition(const std::unordered_map<uint8_t, double> &radians);
  dynamixel::Result<void, dynamixel::DxlError>
  setTargetPosition(double radians);
  dynamixel::Result<void, dynamixel::DxlError> moveToMiddle(uint8_t id);
  dynamixel::Result<double, dynamixel::DxlError> readPosition(uint8_t id);
  dynamixel::Result<std::unordered_map<uint8_t, double>, dynamixel::DxlError>
  readPositions();
  std::unordered_map<uint8_t, double> memPositions() const;

  const std::string &lowLatencyError() const { return low_latency_error_; }

private:
  dynamixel::Motor &servo(uint8_t id);
  dynamixel::Result<void, dynamixel::DxlError>
  setTargetPositionsLocked(const std::unordered_map<uint8_t, double> &radians);

  std::mutex bus_mutex_;
  mutable std::mutex positions_mutex_;
  dynamixel::Connector connector_;
  std::unordered_map<uint8_t, std::unique_ptr<dynamixel::Motor>> servos_;
  std::unordered_map<uint8_t, double> positions_{};
  std::string low_latency_error_;
};

} // namespace payloads
