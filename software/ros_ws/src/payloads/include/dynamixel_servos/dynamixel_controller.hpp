#pragma once

#include <cstdint>
#include <dynamixel_easy_sdk/connector.hpp>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace payloads
{
    /// Thread-safe wrapper over the Dynamixel easy-SDK. Owns the bus and one Motor
    /// per servo found by scan(); one sync read per servo model family and one
    /// sync write serve every servo.
    class DynamixelController
    {
    public:
        using Mode = dynamixel::OperatingMode;
        template <typename T>
        using Result = dynamixel::Result<T, dynamixel::DxlError>;

        static constexpr int32_t counts_per_turn = 4096;
        /// Count at the horn's alignment mark within the first turn.
        static constexpr int32_t center = 2048;
        /// Extended position mode spans +-256 turns.
        static constexpr int32_t count_limit = 1048575;

        DynamixelController(const std::string& device, int baud_rate);
        ~DynamixelController() noexcept;

        Result<std::vector<uint8_t>> scan();
        /// Extended (multi-turn) position mode with no status return delay. EEPROM
        /// is only written when the stored value differs.
        Result<void> configure(uint8_t id);
        Result<void> enableTorque(uint8_t id);
        Result<void> disableTorque(uint8_t id);
        Result<void> setGoalCounts(const std::unordered_map<uint8_t, int32_t>& counts);
        Result<std::unordered_map<uint8_t, int32_t>> readCounts();
        /// Turn index kept in a RAM register (Indirect Address 1): it survives
        /// torque-off restarts and resets with the count on reboot or power loss.
        Result<std::optional<int>> savedTurn(uint8_t id);
        Result<void> saveTurn(uint8_t id, int turn);

        const std::string& lowLatencyError() const { return low_latency_error_; }

    private:
        dynamixel::Motor& servo(uint8_t id);

        std::mutex bus_mutex_;
        dynamixel::Connector connector_;
        std::unordered_map<uint8_t, std::unique_ptr<dynamixel::Motor>> servos_;
        /// Sync Read status packets are returned in ID order but chain only
        /// within one servo firmware family; across families (e.g. MX-106 and
        /// XH430 sharing a bus) every device after the first family boundary
        /// stays silent. Reads are therefore grouped by model number, while the
        /// single sync write stays valid because broadcast writes return no
        /// status packets at all.
        struct ReadGroup
        {
            std::unique_ptr<dynamixel::GroupSyncRead> reader;
            std::vector<uint8_t> ids;
        };
        std::vector<ReadGroup> read_groups_;
        std::unique_ptr<dynamixel::GroupSyncWrite> writer_;
        std::string low_latency_error_;
    };
}  // namespace payloads
