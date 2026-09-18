#include "dynamixel_servos/dynamixel_controller.hpp"

#include <asm/termbits.h>
#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace payloads
{
    namespace
    {
        constexpr uint16_t return_delay_address = 9;
        constexpr uint16_t profile_acceleration_address = 108;
        constexpr uint16_t profile_velocity_address = 112;
        constexpr uint16_t turn_store_address = 168;
        // Odd byte addresses inside the indirect-address block are valid to write
        // and never equal the power-on default (224), so a default reads as unset.
        constexpr int turn_store_base = 169;
        constexpr int turn_store_range = 16;

        // FTDI adapters buffer received bytes until latency_timer ms elapse, 16 ms by
        // default, which alone caps a request/response protocol near 30 Hz. ftdi_sio
        // maps ASYNC_LOW_LATENCY onto that register and drops it to 1 ms; the flag is
        // in ASYNC_USR_MASK so it needs no CAP_SYS_ADMIN. The value is per-port driver
        // state, so a temporary fd suffices. Returns an error description, or empty.
        std::string applyLowLatency(const std::string& device)
        {
            int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd < 0)
            {
                return std::string("open: ") + std::strerror(errno);
            }

            std::string error;
            struct serial_struct serial_info{};
            if (::ioctl(fd, TIOCGSERIAL, &serial_info) != 0)
            {
                error = std::string("TIOCGSERIAL: ") + std::strerror(errno);
            }
            else
            {
                serial_info.flags |= ASYNC_LOW_LATENCY;
                if (::ioctl(fd, TIOCSSERIAL, &serial_info) != 0)
                {
                    error = std::string("TIOCSSERIAL: ") + std::strerror(errno);
                }
            }

            ::close(fd);
            return error;
        }

        // SDK 4.0.3 setupPort() ORs the numeric baud rate into c_cflag, which leaves
        // the line at B0 and every packet unanswered. termios state is per-tty, so
        // re-setting it through a side fd also fixes the SDK's own descriptor.
        std::string applyBaudRate(const std::string& device, int baud_rate)
        {
            int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd < 0)
            {
                return std::string("open: ") + std::strerror(errno);
            }

            std::string error;
            struct termios2 to{};
            if (::ioctl(fd, TCGETS2, &to) != 0)
            {
                error = std::string("TCGETS2: ") + std::strerror(errno);
            }
            else
            {
                to.c_cflag &= ~(CBAUD | CIBAUD | CSIZE | CSTOPB | PARENB);
                to.c_cflag |= BOTHER | CS8 | CLOCAL | CREAD;
                to.c_ispeed = baud_rate;
                to.c_ospeed = baud_rate;
                if (::ioctl(fd, TCSETS2, &to) != 0)
                {
                    error = std::string("TCSETS2: ") + std::strerror(errno);
                }
            }

            ::close(fd);
            return error;
        }

    }  // namespace

    DynamixelController::DynamixelController(const std::string& device, int baud_rate)
        : connector_(device, baud_rate)
    {
        if (const auto err = applyBaudRate(device, baud_rate); !err.empty())
        {
            throw std::runtime_error("Could not set " + std::to_string(baud_rate) +
                                     " baud on " + device + ": " + err);
        }
        low_latency_error_ = applyLowLatency(device);
    }

    DynamixelController::~DynamixelController() noexcept
    {
        for (auto& [id, motor] : servos_)
        {
            (void)id;
            (void)motor->disableTorque();
        }
    }

    auto DynamixelController::scan() -> Result<std::vector<uint8_t>>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        auto result = connector_.broadcastPing();
        if (!result.isSuccess())
        {
            return result.error();
        }
        for (uint8_t id : result.value())
        {
            if (servos_.find(id) == servos_.end())
            {
                servos_.emplace(id, connector_.createMotor(id));
            }
        }
        return result.value();
    }

    auto DynamixelController::configure(uint8_t id) -> Result<void>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        auto& motor = servo(id);

        auto mode = motor.getOperatingMode();
        if (!mode.isSuccess())
        {
            return mode.error();
        }
        if (mode.value() != Mode::EXTENDED_POSITION)
        {
            if (auto off = motor.disableTorque(); !off.isSuccess())
            {
                return off.error();
            }
            if (auto set = motor.setOperatingMode(Mode::EXTENDED_POSITION); !set.isSuccess())
            {
                return set.error();
            }
        }

        auto delay = connector_.read1ByteData(id, return_delay_address);
        if (!delay.isSuccess())
        {
            return delay.error();
        }
        if (delay.value() != 0)
        {
            if (auto off = motor.disableTorque(); !off.isSuccess())
            {
                return off.error();
            }
            if (auto set_delay = connector_.write1ByteData(id, return_delay_address, 0);
                !set_delay.isSuccess())
            {
                return set_delay.error();
            }
        }

        // Ensure internal trajectory profile is set to 0 (unlimited/fastest tracking)
        // so the 100 Hz ros2_control commands track smoothly without internal throttling.
        (void)connector_.write4ByteData(id, profile_acceleration_address, 0);
        (void)connector_.write4ByteData(id, profile_velocity_address, 0);

        return {};
    }

    auto DynamixelController::enableTorque(uint8_t id) -> Result<void>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        return servo(id).enableTorque();
    }

    auto DynamixelController::disableTorque(uint8_t id) -> Result<void>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        return servo(id).disableTorque();
    }

    auto DynamixelController::setGoalCounts(
        const std::unordered_map<uint8_t, int32_t>& counts) -> Result<void>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        if (counts.empty())
        {
            return {};
        }
        auto executor = connector_.createGroupExecutor();
        for (const auto& [id, count] : counts)
        {
            auto command = servo(id).stageSetGoalPosition(
                std::clamp(count, -count_limit, count_limit));
            if (!command.isSuccess())
            {
                return command.error();
            }
            executor->addCmd(command.value());
        }
        return executor->executeWrite();
    }

    auto DynamixelController::readCounts() -> Result<std::unordered_map<uint8_t, int32_t>>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        std::unordered_map<uint8_t, int32_t> counts;
        // An executor with nothing staged has no defined result.
        if (servos_.empty())
        {
            return counts;
        }

        auto executor = connector_.createGroupExecutor();
        std::vector<uint8_t> ids;
        ids.reserve(servos_.size());
        for (auto& [id, motor] : servos_)
        {
            auto command = motor->stageGetPresentPosition();
            if (!command.isSuccess())
            {
                return command.error();
            }
            ids.push_back(id);
            executor->addCmd(command.value());
        }

        auto values = executor->executeRead();
        if (!values.isSuccess())
        {
            return values.error();
        }
        for (std::size_t index = 0; index < ids.size(); ++index)
        {
            auto& value = values.value()[index];
            if (!value.isSuccess())
            {
                return value.error();
            }
            counts.emplace(ids[index], value.value());
        }
        return counts;
    }

    auto DynamixelController::savedTurn(uint8_t id) -> Result<std::optional<int>>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        auto value = connector_.read2ByteData(id, turn_store_address);
        if (!value.isSuccess())
        {
            return value.error();
        }
        int stored = value.value();
        if (stored % 2 == 0 || stored < turn_store_base ||
            stored > turn_store_base + 4 * turn_store_range)
        {
            return std::optional<int>{};
        }
        return std::optional<int>{(stored - turn_store_base) / 2 - turn_store_range};
    }

    auto DynamixelController::saveTurn(uint8_t id, int turn) -> Result<void>
    {
        std::lock_guard<std::mutex> lock(bus_mutex_);
        turn = std::clamp(turn, -turn_store_range, turn_store_range);
        return connector_.write2ByteData(
            id, turn_store_address,
            static_cast<uint16_t>(turn_store_base + 2 * (turn + turn_store_range)));
    }

    dynamixel::Motor& DynamixelController::servo(uint8_t id)
    {
        auto found = servos_.find(id);
        if (found == servos_.end())
        {
            throw std::out_of_range("Servo ID " + std::to_string(id) +
                                    " was not found by scan()");
        }
        return *found->second;
    }

}  // namespace payloads
