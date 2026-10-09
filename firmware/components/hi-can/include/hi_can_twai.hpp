#pragma once

#include <driver/twai.h>

#include <atomic>
#include <board_support.hpp>
#include <chrono>
#include <cstdint>
#include <hi_can.hpp>
#include <optional>
#include <tuple>

namespace hi_can
{
    class TwaiInterface : public FilteredCanInterface
    {
    public:
        static TwaiInterface& get_instance(
            bsp::pin_pair_t pins = std::make_pair(bsp::CAN_TX_PIN, bsp::CAN_RX_PIN),
            uint8_t controller_id = 0, addressing::filter_t filter = {});
        virtual ~TwaiInterface();

        // allow moving but not copying
        TwaiInterface(const TwaiInterface&) = delete;
        TwaiInterface(TwaiInterface&& other) noexcept
            : TwaiInterface()
        {
            swap(*this, other);
        }
        TwaiInterface& operator=(const TwaiInterface&) = delete;
        TwaiInterface& operator=(TwaiInterface&& other) noexcept
        {
            swap(*this, other);
            return *this;
        }

        void transmit(const Packet& packet) override;
        std::optional<Packet> receive(bool blocking = false) override;

        /**
         * @brief Handle error detection and recovery on the underlying TWAI bus
         *
         * This function must be called regularly to ensure that the bus recovers and
         * has errors handled correctly. A bus-off controller is recovered (at most
         * once per CONFIG_HI_CAN_BUS_RECOVERY_INTERVAL ms) and restarted, and frames
         * dropped by transmit() are logged once a second.
         *
         */
        void handle();

        /**
         * @brief Whether any frame has been taken from the bus within the last window_ms
         *
         * Counts every frame receive() gets from the driver, before the software
         * filters. Construct the interface with a zero filter mask to have that be
         * every frame on the bus. Safe to call from any task.
         */
        bool heard_within(uint32_t window_ms) const;

        TwaiInterface& add_filter(const addressing::filter_t& address) override;
        TwaiInterface& remove_filter(const addressing::filter_t& address) override;

        // swap function for move semantics
        friend void swap(TwaiInterface& first, TwaiInterface& second) noexcept
        {
            using std::swap;
            swap(first._controller_id, second._controller_id);
            swap(first._twai_bus, second._twai_bus);
            swap(first._received_packets, second._received_packets);
            swap(first._dropped_frames, second._dropped_frames);
            swap(first._last_recovery, second._last_recovery);
            swap(first._last_drop_log, second._last_drop_log);
            first._last_receive_tick = second._last_receive_tick.exchange(first._last_receive_tick);
        }

    private:
        static constexpr uint8_t INVALID_INTERFACE_ID = 255;
        // Room for a whole burst of periodic transmissions, so a healthy bus never makes transmit() wait.
        static constexpr uint32_t TX_QUEUE_LEN = 16;
        // With a zero filter mask every frame on the bus lands here until receive() takes it.
        static constexpr uint32_t RX_QUEUE_LEN = 64;
        TwaiInterface() = default;  // FOR MOVE SEMANTICS ONLY
        TwaiInterface(bsp::pin_pair_t pins, uint8_t controller_id,
                      addressing::filter_t filter);

        uint8_t _controller_id = INVALID_INTERFACE_ID;
        twai_handle_t _twai_bus = nullptr;  // stays null if the driver install fails

        // Frames transmit() dropped because the bus was off or the TX queue was full.
        uint32_t _dropped_frames = 0;
        std::optional<std::chrono::steady_clock::time_point> _last_recovery;
        std::chrono::steady_clock::time_point _last_drop_log{};
        std::atomic<TickType_t> _last_receive_tick{0};

        std::vector<Packet> _received_packets;
    };
}  // namespace hi_can