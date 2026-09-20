// modbus_rtu.cpp

// TODO: Make some mention of corrupt frames, modbus master will handle this but just for debugging or status might be useful

#include "modbus_rtu.hpp"

#include "pico/time.h"

ModbusRtu::ModbusRtu(Rs485Transport* transport, uint8_t slave_address)
    : transport_(transport),
      slave_address_(slave_address)
{
}

void ModbusRtu::set_read_handler(ReadHoldingRegisterFn fn, void* context)
{
    read_fn_ = fn;
    read_context_ = context;
}

void ModbusRtu::set_write_handler(WriteHoldingRegisterFn fn, void* context)
{
    write_fn_ = fn;
    write_context_ = context;
}

void ModbusRtu::poll()
{
    // Modbus RTU frames are delimited by a silence of at least 3.5
    // character times. At 8N1, one character is 11 bit times (start + 8
    // data + stop): 3.5 * 11 / baud seconds, converted to ms and rounded up.
    uint32_t baud = transport_->baud_hz();
    uint32_t frame_gap_ms = static_cast<uint32_t>((38500ull + baud - 1) / baud);
    if (frame_gap_ms == 0)
        frame_gap_ms = 1;

    uint8_t frame[kMaxFrameLen];
    size_t len = transport_->receive(frame, sizeof(frame), frame_gap_ms);

    if (len < 4)  // shortest possible valid frame: addr + func + 2-byte crc
        return;

    uint16_t received_crc =
        static_cast<uint16_t>(frame[len - 2]) | (static_cast<uint16_t>(frame[len - 1]) << 8);
    uint16_t computed_crc = crc16(frame, len - 2);
    if (received_crc != computed_crc)
        return;  // corrupt frame -- per spec, discard silently, don't reply

    uint8_t addr = frame[0];
    bool broadcast = (addr == 0);
    if (addr != slave_address_ && !broadcast)
        return;  // not addressed to us

    handle_frame(frame, len - 2, broadcast);  // hand off with the CRC stripped
}

void ModbusRtu::note_heartbeat(uint32_t now_ms)
{
    last_heartbeat_ms_ = now_ms;
    heartbeat_seen_ = true;
}

bool ModbusRtu::master_alive(uint32_t now_ms, uint32_t timeout_ms) const
{
    if (!heartbeat_seen_)
        return false;
    return (now_ms - last_heartbeat_ms_) <= timeout_ms;
}

uint16_t ModbusRtu::crc16(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
        {
            if (crc & 0x0001)
                crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
            else
                crc >>= 1;
        }
    }
    return crc;
}

void ModbusRtu::handle_frame(const uint8_t* frame, size_t len, bool broadcast)
{
    uint8_t function = frame[1];
    switch (function)
    {
    case 0x03:
        handle_read_holding_registers(frame, len, broadcast);
        break;
    case 0x06:
        handle_write_single_register(frame, len, broadcast);
        break;
    case 0x10:
        handle_write_multiple_registers(frame, len, broadcast);
        break;
    default:
        send_exception(function, kExceptionIllegalFunction, broadcast);
        break;
    }
}

void ModbusRtu::handle_read_holding_registers(const uint8_t* frame, size_t len, bool broadcast)
{
    if (len != 6)
    {
        send_exception(0x03, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    uint16_t start = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
    uint16_t count = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

    // 5 = addr + func + byte-count bytes; 2 = trailing CRC.
    if (count == 0 || count > (kMaxFrameLen - 5) / 2)
    {
        send_exception(0x03, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    uint8_t response[kMaxFrameLen];
    response[0] = slave_address_;
    response[1] = 0x03;
    response[2] = static_cast<uint8_t>(count * 2);

    for (uint16_t i = 0; i < count; ++i)
    {
        uint16_t value = 0;
        if (!read_fn_ || !read_fn_(static_cast<uint16_t>(start + i), &value, read_context_))
        {
            send_exception(0x03, kExceptionIllegalDataAddress, broadcast);
            return;
        }
        response[3 + i * 2] = static_cast<uint8_t>(value >> 8);
        response[4 + i * 2] = static_cast<uint8_t>(value & 0xFF);
    }

    if (broadcast)
        return;  // spec: never reply to a broadcast request

    send_response(response, 3u + count * 2u);
}

void ModbusRtu::handle_write_single_register(const uint8_t* frame, size_t len, bool broadcast)
{
    if (len != 6)
    {
        send_exception(0x06, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    uint16_t address = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
    uint16_t value = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];

    if (!write_register(address, value))
    {
        send_exception(0x06, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    if (broadcast)
        return;

    // Standard 0x06 response: echo of the request.
    uint8_t response[8];
    for (size_t i = 0; i < 6; ++i)
        response[i] = frame[i];
    send_response(response, 6);
}

void ModbusRtu::handle_write_multiple_registers(const uint8_t* frame, size_t len, bool broadcast)
{
    if (len < 7)
    {
        send_exception(0x10, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    uint16_t start = (static_cast<uint16_t>(frame[2]) << 8) | frame[3];
    uint16_t count = (static_cast<uint16_t>(frame[4]) << 8) | frame[5];
    uint8_t byte_count = frame[6];

    if (count == 0 || byte_count != count * 2 || len != static_cast<size_t>(7 + byte_count))
    {
        send_exception(0x10, kExceptionIllegalDataAddress, broadcast);
        return;
    }

    for (uint16_t i = 0; i < count; ++i)
    {
        uint16_t value =
            (static_cast<uint16_t>(frame[7 + i * 2]) << 8) | frame[8 + i * 2];
        if (!write_register(static_cast<uint16_t>(start + i), value))
        {
            send_exception(0x10, kExceptionIllegalDataAddress, broadcast);
            return;
        }
    }

    if (broadcast)
        return;

    uint8_t response[8];
    response[0] = slave_address_;
    response[1] = 0x10;
    response[2] = frame[2];
    response[3] = frame[3];
    response[4] = frame[4];
    response[5] = frame[5];
    send_response(response, 6);
}

bool ModbusRtu::write_register(uint16_t address, uint16_t value)
{
    // kRegHeartbeat is protocol-level bookkeeping this class owns directly
    // -- it doesn't go through the application's write_fn_ like other
    // registers do.
    if (address == kRegHeartbeat)
    {
        note_heartbeat(to_ms_since_boot(get_absolute_time()));
        return true;
    }
    if (!write_fn_)
        return false;
    return write_fn_(address, value, write_context_);
}

void ModbusRtu::send_exception(uint8_t function, ModbusException exception, bool broadcast)
{
    if (broadcast)
        return;  // spec: never reply to broadcast, not even with an exception

    uint8_t response[5];
    response[0] = slave_address_;
    response[1] = function | 0x80;
    response[2] = static_cast<uint8_t>(exception);
    send_response(response, 3);
}

void ModbusRtu::send_response(uint8_t* frame, size_t len)
{
    uint16_t crc = crc16(frame, len);
    frame[len] = static_cast<uint8_t>(crc & 0xFF);
    frame[len + 1] = static_cast<uint8_t>(crc >> 8);
    transport_->send(frame, len + 2);
}
