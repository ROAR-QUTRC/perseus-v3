// modbus_rtu.hpp
//
// Modbus RTU slave framing on top of an Rs485Transport.

#pragma once

#include <cstddef>
#include <cstdint>

#include "rs485_transport.hpp"

// Holding register map. All registers are 16-bit; function codes 0x03
// (read) / 0x06 (write single) / 0x10 (write multiple) operate on these
// addresses.
enum ModbusRegister : uint16_t
{
    kRegAngleRaw = 0,         // read-only, AS5600 counts (0-4095) after zero offset
    kRegAngleDegreesX10 = 1,  // read-only, degrees * 10 (0-3599), for convenience
    kRegZeroCommand = 2,      // write-only: any nonzero value zeroes the current angle
    kRegHeartbeat = 3,        // write-only: master writes periodically; see master_alive()
    kRegStatus = 4,           // read-only bitfield: bit0 magnet_detected, bit1 master_alive
    kRegDiscovery = 5,        // read/write: nonzero = solid white identify beacon, 0 = off
};

// Standard Modbus exception codes (returned in the response's data byte
// when the high bit of the function code is set).
enum ModbusException : uint8_t
{
    kExceptionIllegalFunction = 0x01,
    kExceptionIllegalDataAddress = 0x02,
    kExceptionSlaveDeviceFailure = 0x04,
};

// Slave address is the DIP-switch device ID (0-7, see board_id.hpp) plus
// one, giving 1-8. Real Modbus RTU reserves address 0 for broadcast (every
// slave executes the request, none reply), so device ID 0 ("all off" on
// the DIP switch) still needs a nonzero address to be individually
// addressable -- the +1 offset is applied by whoever constructs this
// class (e.g. ModbusRtu(&transport, read_board_id() + 1)), not inside
// board_id.cpp, since read_board_id()'s raw 0-7 value is also used
// elsewhere (the status LED's ack-chase slot).
class ModbusRtu
{
public:
    // Callback hooks the application layer provides for register access.
    // Return false to have the slave respond with a Modbus exception.
    using ReadHoldingRegisterFn = bool (*)(uint16_t address, uint16_t* out_value, void* context);
    using WriteHoldingRegisterFn = bool (*)(uint16_t address, uint16_t value, void* context);

    ModbusRtu(Rs485Transport* transport, uint8_t slave_address);

    void set_read_handler(ReadHoldingRegisterFn fn, void* context);
    void set_write_handler(WriteHoldingRegisterFn fn, void* context);

    // Services one incoming request if one is waiting, replies over the
    // transport. Meant to be called regularly from the communications
    // task/core; each call blocks for up to ~3.5 character times waiting
    // to see whether a frame is arriving (see Rs485Transport::receive()).
    void poll();

    // Called internally when kRegHeartbeat is written (and available to
    // call directly for testing). `now_ms` is expected to be a monotonic
    // millis-since-boot count, e.g. to_ms_since_boot(get_absolute_time()).
    void note_heartbeat(uint32_t now_ms);

    // True if a heartbeat write landed within timeout_ms of now_ms. Meant
    // to be polled by the status-LED task so a board can show "lost comms"
    // even if the master has stopped addressing it entirely -- this is
    // independent of (and complements) the master tracking per-slave
    // response timeouts on its own side.
    bool master_alive(uint32_t now_ms, uint32_t timeout_ms) const;

    // Distinguishes "never got a heartbeat yet" from "got one, timed out"
    // -- both make master_alive() false, but the status LED shows a
    // different pattern for each (see status_led.hpp).
    bool heartbeat_ever_seen() const { return heartbeat_seen_; }

    // Timestamp of the last heartbeat write, for StatusLed::update_heartbeat_state().
    uint32_t last_heartbeat_ms() const { return last_heartbeat_ms_; }

    // CRC16 (Modbus polynomial 0xA001).
    static uint16_t crc16(const uint8_t* data, size_t len);

private:
    static constexpr size_t kMaxFrameLen = 64;  // far more than this register map ever needs

    void handle_frame(const uint8_t* frame, size_t len, bool broadcast);
    void handle_read_holding_registers(const uint8_t* frame, size_t len, bool broadcast);
    void handle_write_single_register(const uint8_t* frame, size_t len, bool broadcast);
    void handle_write_multiple_registers(const uint8_t* frame, size_t len, bool broadcast);
    void send_exception(uint8_t function, ModbusException exception, bool broadcast);
    void send_response(uint8_t* frame, size_t len);         // appends CRC, sends
    bool write_register(uint16_t address, uint16_t value);  // routes kRegHeartbeat specially

    Rs485Transport* transport_;
    uint8_t slave_address_;
    ReadHoldingRegisterFn read_fn_ = nullptr;
    void* read_context_ = nullptr;
    WriteHoldingRegisterFn write_fn_ = nullptr;
    void* write_context_ = nullptr;

    uint32_t last_heartbeat_ms_ = 0;
    bool heartbeat_seen_ = false;  // false until the first write, so we don't start out "alive"
};
