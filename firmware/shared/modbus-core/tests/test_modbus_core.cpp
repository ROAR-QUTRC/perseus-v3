// Host-side tests: a real Master talks to real Slaves over an in-memory bus
// with a fake clock and fault injection. No hardware involved.

#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <vector>

#include "modbus/heartbeat.hpp"
#include "modbus/master.hpp"
#include "modbus/mbus.hpp"
#include "modbus/profiles/encoder.hpp"
#include "modbus/slave.hpp"

static int g_failures = 0;
#define CHECK(cond)                                                     \
    do                                                                  \
    {                                                                   \
        if (!(cond))                                                    \
        {                                                               \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                               \
        }                                                               \
    } while (0)

using namespace modbus;

class Endpoint;

struct Bus
{
    uint32_t now_us = 0;
    uint32_t baud = 115200;
    std::vector<Endpoint*> endpoints;
    bool corrupt_next_request = false;
    bool corrupt_next_response = false;
    bool drop_next_response = false;
};

// Master transmissions reach every slave; slave transmissions reach only the master.
class Endpoint : public rs485::Port, public Clock
{
public:
    Endpoint(Bus& bus, bool is_master)
        : bus_(bus),
          is_master_(is_master)
    {
        bus.endpoints.push_back(this);
    }

    std::deque<uint8_t> inbound;
    std::function<void()> on_data;  // slaves set this to run poll()
    bool present = true;            // false simulates a cut cable
    uint32_t frames_sent = 0;

    bool send(const uint8_t* data, size_t len) override
    {
        ++frames_sent;
        bus_.now_us += static_cast<uint32_t>(len * 11ull * 1000000ull / bus_.baud);

        std::vector<uint8_t> bytes(data, data + len);
        if (is_master_ && bus_.corrupt_next_request)
        {
            bytes[len / 2] ^= 0x55;
            bus_.corrupt_next_request = false;
        }
        if (!is_master_)
        {
            if (bus_.drop_next_response)
            {
                bus_.drop_next_response = false;
                return true;
            }
            if (bus_.corrupt_next_response)
            {
                bytes[1] ^= 0x55;
                bus_.corrupt_next_response = false;
            }
        }

        for (Endpoint* other : bus_.endpoints)
            if (other != this && other->is_master_ != is_master_ && other->present)
                for (uint8_t b : bytes)
                    other->inbound.push_back(b);
        for (Endpoint* other : bus_.endpoints)
            if (other != this && other->is_master_ != is_master_ && other->present && other->on_data)
                other->on_data();
        return true;
    }

    size_t receive(uint8_t* buf, size_t cap, uint32_t first_byte_timeout_us, uint32_t) override
    {
        if (inbound.empty())
        {
            if (is_master_)
                bus_.now_us += first_byte_timeout_us;
            return 0;
        }
        size_t n = 0;
        while (n < cap && !inbound.empty())
        {
            buf[n++] = inbound.front();
            inbound.pop_front();
        }
        return n;
    }

    void flush_rx() override { inbound.clear(); }
    uint32_t now_ms() override { return bus_.now_us / 1000; }
    uint32_t now_us() override { return bus_.now_us; }
    void delay_us(uint32_t us) override { bus_.now_us += us; }
    uint32_t baud_hz() const override { return bus_.baud; }

private:
    Bus& bus_;
    bool is_master_;
};

// A slave with a tiny generic register file: 0, 1 and 4 are read-only, 2 and 3
// are write-only, 5 is read/write, and anything else is illegal.
struct FakeDevice
{
    Endpoint ep;
    Slave slave;
    uint16_t regs[6] = {};
    int writes = 0;
    uint16_t last_write_reg = 0xFFFF;
    uint16_t last_write_value = 0;

    FakeDevice(Bus& bus, uint8_t address)
        : ep(bus, false),
          slave(ep, address)
    {
        slave.set_read_handler(&FakeDevice::read, this);
        slave.set_write_handler(&FakeDevice::write, this);
        ep.on_data = [this]
        { slave.poll(); };
    }

    static bool read(uint16_t address, uint16_t* out, void* ctx)
    {
        auto* d = static_cast<FakeDevice*>(ctx);
        if (address >= 6 || address == 2 || address == 3)
            return false;
        *out = d->regs[address];
        return true;
    }

    static bool write(uint16_t address, uint16_t value, void* ctx)
    {
        auto* d = static_cast<FakeDevice*>(ctx);
        if (address != 2 && address != 3 && address != 5)
            return false;
        d->regs[address] = value;
        d->last_write_reg = address;
        d->last_write_value = value;
        ++d->writes;
        return true;
    }
};

static void test_crc_and_framing()
{
    const uint8_t frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    CHECK(crc16(frame, sizeof(frame)) == 0xCDC5);  // known Modbus vector: C5 CD on the wire

    uint8_t out[8];
    const size_t n = build_read_request(out, 1, 0, 10);
    CHECK(n == 8);
    CHECK(out[6] == 0xC5 && out[7] == 0xCD);
    CHECK(crc_valid(out, n));
    out[3] ^= 1;
    CHECK(!crc_valid(out, n));

    CHECK(frame_gap_us(9600) == 4011);  // 38.5 bit times, rounded up
    CHECK(frame_gap_us(115200) == 335);
}

static void test_master_slave_basics()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice dev(bus, 4);
    Master master(m, m, 10);
    dev.regs[0] = 1234;
    dev.regs[1] = 1805;

    Response r = master.read_holding(4, 0, 2);
    CHECK(r.result == Result::Ok);
    CHECK(r.reg_count == 2 && r.regs[0] == 1234 && r.regs[1] == 1805);
    CHECK(r.latency_us > 0);

    r = master.read_holding(4, 2, 1);  // write-only register
    CHECK(r.result == Result::ExceptionReply && r.exception_code == 0x02);

    r = master.read_holding(4, 5, 3);  // runs off the end of the map
    CHECK(r.result == Result::ExceptionReply && r.exception_code == 0x02);

    r = master.write_single(4, 5, 1);
    CHECK(r.result == Result::Ok);
    CHECK(dev.writes == 1 && dev.last_write_reg == 5 && dev.last_write_value == 1);

    r = master.write_single(4, 0, 1);  // read-only register
    CHECK(r.result == Result::ExceptionReply && r.exception_code == 0x02);

    r = master.read_holding(4, 0, 0);
    CHECK(r.result == Result::InvalidRequest);
    r = master.read_holding(kBroadcastAddress, 0, 1);
    CHECK(r.result == Result::InvalidRequest);
}

static void test_broadcast()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice a(bus, 4);
    FakeDevice b(bus, 5);
    Master master(m, m, 10);
    a.regs[0] = 7;

    Response r = master.write_single(kBroadcastAddress, 3, 1);
    CHECK(r.result == Result::Ok);
    CHECK(a.writes == 1 && b.writes == 1);
    CHECK(a.ep.frames_sent == 0 && b.ep.frames_sent == 0);  // nobody replies to a broadcast

    // No manual delay needed afterwards: the master already waited out the gap.
    r = master.read_holding(4, 0, 1);
    CHECK(r.result == Result::Ok && r.regs[0] == 7);
}

static void test_faults()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice dev(bus, 4);
    Master master(m, m, 10);

    bus.corrupt_next_response = true;
    Response r = master.read_holding(4, 0, 1);
    CHECK(r.result == Result::CrcError);
    CHECK(master.read_holding(4, 0, 1).result == Result::Ok);

    bus.drop_next_response = true;
    const uint32_t before = bus.now_us;
    r = master.read_holding(4, 0, 1);
    CHECK(r.result == Result::Timeout);
    CHECK(bus.now_us - before >= 10000);  // waited the full response timeout

    bus.corrupt_next_request = true;
    r = master.read_holding(4, 0, 1);
    CHECK(r.result == Result::Timeout);  // slave discards it silently
    CHECK(dev.slave.counters().crc_errors == 1);

    r = master.read_holding(9, 0, 1);  // nobody home
    CHECK(r.result == Result::Timeout);
}

struct StateLog
{
    int changes = 0;
    DeviceState from = DeviceState::Unknown;
    DeviceState to = DeviceState::Unknown;
};

static void on_state(uint8_t, DeviceState from, DeviceState to, void* ctx)
{
    auto* log = static_cast<StateLog*>(ctx);
    ++log->changes;
    log->from = from;
    log->to = to;
}

template <typename Sched, typename Pred>
static bool run_until(Bus& bus, Sched& sched, Pred done, int max_steps = 3000)
{
    for (int i = 0; i < max_steps; ++i)
    {
        if (done())
            return true;
        if (!sched.step())
            bus.now_us += 1000;
    }
    return done();
}

static Device make_test_device(uint8_t slave, StateLog* log, bool latch = false)
{
    Device d;
    d.slave = slave;
    d.on_state_change = &on_state;
    d.ctx = log;
    d.policy.latch_degraded = latch;
    PollJob job;
    job.start_reg = 0;
    job.count = 2;
    job.period_ms = 20;
    d.add_job(job);
    return d;
}

static void test_scheduler_polling_and_health()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice d1(bus, 1), d2(bus, 2), d3(bus, 3);
    StateLog l1, l2, l3;
    MBUS<3, 4> sched(m, m, 10);
    CHECK(sched.add_device(make_test_device(1, &l1)));
    CHECK(sched.add_device(make_test_device(2, &l2)));
    CHECK(sched.add_device(make_test_device(3, &l3)));
    CHECK(!sched.add_device(make_test_device(3, &l3)));  // duplicate address

    DeviceStats s1, s2;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(3, &s1) && s1.total >= 5; }));
    for (uint8_t id = 1; id <= 3; ++id)
    {
        CHECK(sched.get_stats(id, &s1));
        CHECK(s1.state == DeviceState::Ok);
        CHECK(s1.timeouts == 0 && s1.consecutive_failures == 0);
        CHECK(s1.total >= 4);
    }
    CHECK(l1.changes == 1 && l1.to == DeviceState::Ok);  // Unknown -> Ok exactly once

    // Cut device 2's cable: two consecutive failures -> Degraded, ten -> Lost.
    d2.ep.present = false;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(2, &s2) && s2.state == DeviceState::Degraded; }));
    CHECK(s2.consecutive_failures == 2);
    CHECK(l2.from == DeviceState::Ok && l2.to == DeviceState::Degraded);
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(2, &s2) && s2.state == DeviceState::Lost; }));
    CHECK(s2.consecutive_failures == 10);

    // The neighbours on the same bus are unaffected.
    CHECK(sched.get_stats(1, &s1) && s1.state == DeviceState::Ok);
    CHECK(sched.get_stats(3, &s1) && s1.state == DeviceState::Ok);

    // Reconnect: Lost -> Degraded on the first good reply, then back to Ok.
    d2.ep.present = true;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(2, &s2) && s2.state == DeviceState::Degraded; }));
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(2, &s2) && s2.state == DeviceState::Ok; }));
    CHECK(s2.timeouts >= 10);
}

static void test_latch()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice d1(bus, 1);
    StateLog log;
    MBUS<1, 2> sched(m, m, 10);
    sched.add_device(make_test_device(1, &log, /*latch=*/true));

    DeviceStats s;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(1, &s) && s.state == DeviceState::Ok; }));
    d1.ep.present = false;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(1, &s) && s.state == DeviceState::Degraded; }));
    d1.ep.present = true;

    // Healthy again, but the latch holds it at Degraded.
    for (int i = 0; i < 300; ++i)
        if (!sched.step())
            bus.now_us += 1000;
    CHECK(sched.get_stats(1, &s) && s.state == DeviceState::Degraded && s.latched);

    CHECK(sched.clear_latch(1));
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(1, &s) && s.state == DeviceState::Ok; }));
}

static void test_exception_is_not_a_link_failure()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice d1(bus, 1);
    StateLog log;
    MBUS<1, 2> sched(m, m, 10);

    Device d = make_test_device(1, &log);
    d.jobs[0].start_reg = 2;  // write-only register -> the board replies with an exception every time
    d.jobs[0].count = 1;
    sched.add_device(d);

    DeviceStats s;
    CHECK(run_until(bus, sched, [&]
                    { return sched.get_stats(1, &s) && s.total >= 30; }));
    CHECK(s.exceptions == s.total);
    CHECK(s.state == DeviceState::Ok && s.consecutive_failures == 0 && s.timeouts == 0);
}

struct Done
{
    int calls = 0;
    Result result = Result::Timeout;
};

static void on_done(const Response& r, void* ctx)
{
    auto* d = static_cast<Done*>(ctx);
    ++d->calls;
    d->result = r.result;
}

static void test_one_shot_requests()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice d1(bus, 1), d2(bus, 2);
    StateLog l1, l2;
    MBUS<2, 4> sched(m, m, 10);
    sched.add_device(make_test_device(1, &l1));
    sched.add_device(make_test_device(2, &l2));

    // A targeted write completes and reports back.
    Done zero;
    Request req;
    req.slave = 1;
    req.kind = Request::Kind::WriteReg;
    req.reg = 2;
    req.value_or_count = 1;
    req.on_done = &on_done;
    req.ctx = &zero;
    CHECK(sched.submit(req));
    CHECK(run_until(bus, sched, [&]
                    { return zero.calls == 1; }));
    CHECK(zero.result == Result::Ok);
    CHECK(d1.last_write_reg == 2 && d1.writes == 1 && d2.writes == 0);

    // A broadcast heartbeat reaches everyone and touches no device's stats.
    DeviceStats before, after;
    sched.get_stats(1, &before);
    Done hb;
    req.slave = kBroadcastAddress;
    req.reg = 3;
    req.on_done = &on_done;
    req.ctx = &hb;
    CHECK(sched.submit(req));
    const uint32_t polls_before = before.total;
    CHECK(run_until(bus, sched, [&]
                    { return hb.calls == 1; }));
    CHECK(hb.result == Result::Ok && d1.writes == 2 && d2.writes == 1);
    sched.get_stats(1, &after);
    CHECK(after.total <= polls_before + 2);  // only ordinary polls, none for the broadcast

    // A request to an unregistered address still completes (e.g. a discovery scan).
    Done scan;
    req.slave = 9;
    req.kind = Request::Kind::ReadRegs;
    req.reg = 0;
    req.value_or_count = 1;
    req.retries = 1;
    req.on_done = &on_done;
    req.ctx = &scan;
    CHECK(sched.submit(req));
    CHECK(run_until(bus, sched, [&]
                    { return scan.calls == 1; }));
    CHECK(scan.result == Result::Timeout);

    // The queue rejects overflow rather than growing.
    int accepted = 0;
    for (int i = 0; i < 10; ++i)
        accepted += sched.submit(req) ? 1 : 0;
    CHECK(accepted == 4);
}

struct EncoderSeen
{
    modbus::profiles::encoder::Angle angle;
    modbus::profiles::encoder::Status status;
    int angle_updates = 0;
    int status_updates = 0;
};

static void on_encoder_angle(const Response& r, void* ctx)
{
    auto* seen = static_cast<EncoderSeen*>(ctx);
    if (modbus::profiles::encoder::decode_angle(r, &seen->angle))
        ++seen->angle_updates;
}

static void on_encoder_status(const Response& r, void* ctx)
{
    auto* seen = static_cast<EncoderSeen*>(ctx);
    if (modbus::profiles::encoder::decode_status(r, &seen->status))
        ++seen->status_updates;
}

static void test_encoder_profile()
{
    namespace enc = modbus::profiles::encoder;

    Bus bus;
    Endpoint m(bus, true);
    FakeDevice board(bus, 4);
    board.regs[enc::kRegAngleRaw] = 2048;
    board.regs[enc::kRegAngleDegreesX10] = 1800;
    board.regs[enc::kRegStatus] = enc::kStatusMagnetDetected;

    EncoderSeen seen;
    enc::DeviceConfig cfg;
    cfg.slave = 4;
    cfg.on_angle = &on_encoder_angle;
    cfg.on_status = &on_encoder_status;
    cfg.ctx = &seen;

    MBUS<1, 4> sched(m, m, 10);
    CHECK(sched.add_device(enc::make_device(cfg)));
    CHECK(run_until(bus, sched, [&]
                    { return seen.angle_updates >= 3 && seen.status_updates >= 1; }));
    CHECK(seen.angle.raw_counts == 2048 && seen.angle.degrees_x10 == 1800);
    CHECK(seen.status.magnet_detected && !seen.status.master_alive);

    // The status job runs slower than the angle job.
    CHECK(seen.angle_updates > seen.status_updates);

    // Status bits decode independently.
    board.regs[enc::kRegStatus] = enc::kStatusMasterAlive;
    const int before = seen.status_updates;
    CHECK(run_until(bus, sched, [&]
                    { return seen.status_updates > before; }));
    CHECK(!seen.status.magnet_detected && seen.status.master_alive);

    // The request builders hit the right registers.
    CHECK(sched.submit(enc::zero_request(4)));
    CHECK(run_until(bus, sched, [&]
                    { return board.writes == 1; }));
    CHECK(board.last_write_reg == enc::kRegZeroCommand && board.last_write_value != 0);

    CHECK(sched.submit(enc::discovery_request(4, true)));
    CHECK(run_until(bus, sched, [&]
                    { return board.writes == 2; }));
    CHECK(board.last_write_reg == enc::kRegDiscovery && board.last_write_value == 1);

    CHECK(sched.submit(enc::discovery_request(4, false)));
    CHECK(run_until(bus, sched, [&]
                    { return board.writes == 3; }));
    CHECK(board.last_write_value == 0);

    CHECK(sched.submit(enc::heartbeat_request()));
    CHECK(run_until(bus, sched, [&]
                    { return board.writes == 4; }));
    CHECK(board.last_write_reg == enc::kRegHeartbeat);
}

// The byte-for-byte example in README.md and the encoder's docs/modbus.md; keep them in step.
static void test_documented_angle_frames()
{
    Bus bus;
    Endpoint m(bus, true);
    FakeDevice dev(bus, 1);
    dev.regs[0] = 2048;
    dev.regs[1] = 1800;

    const uint8_t request[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B};
    uint8_t built[8];
    CHECK(build_read_request(built, 1, 0, 2) == sizeof(request));
    CHECK(std::memcmp(built, request, sizeof(request)) == 0);

    m.send(request, sizeof(request));
    const std::vector<uint8_t> reply(m.inbound.begin(), m.inbound.end());
    const std::vector<uint8_t> expected_reply = {0x01, 0x03, 0x04, 0x08, 0x00, 0x07, 0x08, 0xFB, 0xA5};
    CHECK(reply == expected_reply);

    // Any rejected register (here the write-only registers 2-3) gives the same exception frame.
    m.inbound.clear();
    const size_t n = build_read_request(built, 1, 2, 2);
    m.send(built, n);
    const std::vector<uint8_t> expected_exception = {0x01, 0x83, 0x02, 0xC0, 0xF1};
    CHECK(std::vector<uint8_t>(m.inbound.begin(), m.inbound.end()) == expected_exception);
}

static void test_heartbeat_tracker()
{
    HeartbeatTracker hb;
    CHECK(!hb.alive(1000, 3000) && !hb.ever_seen());
    hb.note(1000);
    CHECK(hb.alive(3999, 3000) && hb.ever_seen());
    CHECK(!hb.alive(4001, 3000) && hb.ever_seen());
}

int main()
{
    test_crc_and_framing();
    test_master_slave_basics();
    test_broadcast();
    test_faults();
    test_scheduler_polling_and_health();
    test_latch();
    test_exception_is_not_a_link_failure();
    test_one_shot_requests();
    test_encoder_profile();
    test_documented_angle_frames();
    test_heartbeat_tracker();

    if (g_failures == 0)
        std::printf("all tests passed\n");
    else
        std::printf("%d check(s) failed\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}
