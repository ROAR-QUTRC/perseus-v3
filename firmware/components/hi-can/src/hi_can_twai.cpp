#include "hi_can_twai.hpp"

#include <driver/gpio.h>
#include <unistd.h>

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static const char* const TAG = "hi_can_twai";

using namespace bsp;
using namespace hi_can;
using namespace std::chrono;
using namespace std::chrono_literals;
using std::string;

TwaiInterface& TwaiInterface::get_instance(pin_pair_t pins,
                                           uint8_t controller_id,
                                           addressing::filter_t filter)
{
    static std::optional<TwaiInterface> instance;
    if (!instance)
        instance = TwaiInterface(pins, controller_id, filter);
    return *instance;
}

TwaiInterface::TwaiInterface(pin_pair_t pins, uint8_t controller_id,
                             addressing::filter_t filter)
    : _controller_id(controller_id)
{
#ifndef CONFIG_HI_CAN_NO_ACK
    // standard configs at 125kbps
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(
        controller_id, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NORMAL);
#else
    // loopback - not going to get an ACK
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT_V2(
        controller_id, std::get<0>(pins), std::get<1>(pins), TWAI_MODE_NO_ACK);
#endif

    general_config.tx_queue_len = TX_QUEUE_LEN;
    general_config.rx_queue_len = RX_QUEUE_LEN;

#if defined(CONFIG_HI_CAN_BAUD_1M)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
#elif defined(CONFIG_HI_CAN_BAUD_250K)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_250KBITS();
#elif defined(CONFIG_HI_CAN_BAUD_125K)
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_125KBITS();
#else  // default: 500K
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_500KBITS();
#endif

    // A zero mask cares about no address bits: every frame on the bus is accepted.
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (filter.mask != 0)
    {
        filter_config = {
            .acceptance_code = filter.address.address << 3,
            // TWAI filter requires MSB to be rightmost, and RTR filtering is bit 3
            // Additionally, the acceptance mask is "set bit to ignore" rather than
            // "clear to ignore" like SocketCAN
            .acceptance_mask = ((~filter.mask) << 3) | 0x00000004,
            .single_filter = true,
        };
    }

    if (twai_driver_install_v2(&general_config, &timing_config, &filter_config,
                               &_twai_bus) == ESP_OK)
    {
        printf("Driver installed\n");
    }
    else
    {
        printf("Failed to install driver\n");
        return;
    }
    if (twai_start_v2(_twai_bus) == ESP_OK)
    {
        printf("Driver started\n");
    }
    else
    {
        printf("Failed to start driver\n");
        return;
    }
    if (twai_reconfigure_alerts_v2(_twai_bus, TWAI_ALERT_ALL, nullptr) ==
        ESP_OK)
    {
        printf("Alerts reconfigured\n");
    }
    else
    {
        printf("Failed to reconfigure alerts\n");
        return;
    }
}

TwaiInterface::~TwaiInterface()
{
    if (_controller_id == INVALID_INTERFACE_ID)
        return;
    twai_stop_v2(_twai_bus);
    twai_driver_uninstall_v2(_twai_bus);
}
void TwaiInterface::transmit(const Packet& packet)
{
    const auto address = packet.get_address();
    twai_message_t message{
        .identifier = address.address,
        .data_length_code = static_cast<uint8_t>(packet.get_data_len()),
    };
    std::copy_n(packet.get_data().cbegin(), packet.get_data_len(), message.data);
    message.extd = address.is_extended;
    message.rtr = address.is_rtr;
    message.ss = false;            // not single-shot (re-try on error)
    message.self = false;          // not self-reception
    message.dlc_non_comp = false;  // data length code is <= 8

    const esp_err_t err = twai_transmit_v2(
        _twai_bus, &message, pdMS_TO_TICKS(CONFIG_HI_CAN_BUS_TX_TIME));
    if (err == ESP_OK)
        return;

    // TIMEOUT: TX queue full (busy bus, or no ACK in normal mode).
    // INVALID_STATE: bus-off or recovering. Both are faults on the bus, not in
    // the caller, so the frame is dropped; handle() logs the count.
    if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE)
    {
        ++_dropped_frames;
        return;
    }
    throw std::runtime_error(
        std::format("Failed to transmit packet {:#08x}: {}",
                    packet.get_address().address, esp_err_to_name(err)));
}
std::optional<Packet> TwaiInterface::receive(bool blocking)
{
    twai_message_t message;
    if (esp_err_t err =
            twai_receive_v2(_twai_bus, &message, blocking ? portMAX_DELAY : 0);
        err != ESP_OK)
    {
        if (err == ESP_ERR_TIMEOUT)
            return std::nullopt;
        if (err == ESP_ERR_INVALID_ARG)
            throw std::runtime_error("Invalid arguments for TWAI receive");
        if (err == ESP_ERR_INVALID_STATE)
            throw std::runtime_error("TWAI driver not installed");
    }
    _last_receive_tick = xTaskGetTickCount();

    Packet packet{addressing::flagged_address_t(message.identifier, message.rtr,
                                                false, message.extd),
                  message.data, message.data_length_code};

    if (_receive_callback)
        _receive_callback(packet);

    return packet;
}

bool TwaiInterface::heard_within(uint32_t window_ms) const
{
    // Read before the clock: the other way round, a frame arriving in between
    // would make the difference wrap.
    const TickType_t heard = _last_receive_tick;
    return xTaskGetTickCount() - heard <= pdMS_TO_TICKS(window_ms);
}

// Works from the driver state rather than alerts, so a missed alert can't
// leave the bus stuck.
void TwaiInterface::handle()
{
    if (_twai_bus == nullptr)
        return;

    twai_status_info_t status;
    if (twai_get_status_info_v2(_twai_bus, &status) != ESP_OK)
        return;

    const auto now = steady_clock::now();
    switch (status.state)
    {
    case TWAI_STATE_BUS_OFF:
        // Straight away the first time; a node that keeps going bus-off retries
        // once per interval, so a wiring fault can't flood the bus with error frames.
        if ((!_last_recovery ||
             now - *_last_recovery >= milliseconds(CONFIG_HI_CAN_BUS_RECOVERY_INTERVAL)) &&
            twai_initiate_recovery_v2(_twai_bus) == ESP_OK)
        {
            _last_recovery = now;
            ESP_LOGW(TAG, "bus-off (TEC %lu), recovering",
                     static_cast<unsigned long>(status.tx_error_counter));
        }
        break;
    case TWAI_STATE_STOPPED:
        // Where the driver parks after a recovery; it won't transmit until restarted.
        if (twai_start_v2(_twai_bus) == ESP_OK)
            ESP_LOGI(TAG, "bus recovered, driver restarted");
        break;
    default:  // RUNNING, or RECOVERING (waiting for 128 idle periods on the bus)
        break;
    }

    if (_dropped_frames > 0 && now - _last_drop_log >= 1s)
    {
        ESP_LOGW(TAG, "dropped %lu frames; TEC %lu, REC %lu, %lu bus errors",
                 static_cast<unsigned long>(_dropped_frames),
                 static_cast<unsigned long>(status.tx_error_counter),
                 static_cast<unsigned long>(status.rx_error_counter),
                 static_cast<unsigned long>(status.bus_error_count));
        _dropped_frames = 0;
        _last_drop_log = now;
    }
}

TwaiInterface&
hi_can::TwaiInterface::add_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::add_filter(address);
    // TODO: IMPLEMENT
    return *this;
}

TwaiInterface&
hi_can::TwaiInterface::remove_filter(const addressing::filter_t& address)
{
    FilteredCanInterface::remove_filter(address);
    // TODO: IMPLEMENT
    return *this;
}
