#pragma once

#include <cstddef>
#include <cstdint>

#include "modbus/clock.hpp"
#include "modbus/protocol.hpp"
#include "rs485/port.hpp"

namespace modbus
{
    enum class Result : uint8_t
    {
        Ok,
        Timeout,         // nothing came back
        CrcError,        // something came back, but garbled
        ExceptionReply,  // the slave answered with a Modbus exception
        BadFrame,        // valid CRC but wrong address, function or length
        InvalidRequest,  // rejected locally, nothing was sent
        TransportError,  // the transport failed to send
    };

    inline constexpr size_t kMaxResponseRegs = 16;

    struct Response
    {
        Result result = Result::Timeout;
        uint8_t exception_code = 0;  // valid when result == ExceptionReply
        uint8_t reg_count = 0;
        uint16_t regs[kMaxResponseRegs] = {};
        uint32_t timestamp_ms = 0;  // when the transaction finished
        uint32_t latency_us = 0;    // request start to reply received
    };

    // Blocking Modbus RTU master: one call is one whole transaction on the bus.
    // Not thread safe; exactly one task should own a Master (and its bus).
    class Master
    {
    public:
        explicit Master(rs485::Port& port, Clock& clock, uint32_t response_timeout_ms = 50)
            : port_(port),
              clock_(clock),
              response_timeout_us_(response_timeout_ms * 1000u)
        {
        }

        Response read_holding(uint8_t slave, uint16_t start, uint16_t count);

        // slave == kBroadcastAddress writes to every slave and waits for no reply.
        Response write_single(uint8_t slave, uint16_t reg, uint16_t value);

    private:
        // Sends `request`, waits for a reply and classifies it. On Result::Ok the
        // reply frame is left in rx/rx_len for the caller to parse.
        Response exchange(const uint8_t* request, size_t request_len, uint8_t slave, uint8_t function,
                          uint8_t* rx, size_t* rx_len);

        rs485::Port& port_;
        Clock& clock_;
        uint32_t response_timeout_us_;
    };
}  // namespace modbus
