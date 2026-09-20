#pragma once

#include <cstddef>
#include <cstdint>

#include "modbus/master.hpp"

namespace modbus
{
    using ResultFn = void (*)(const Response& response, void* ctx);

    // A standing order: read `count` registers from a device every `period_ms`.
    struct PollJob
    {
        uint16_t start_reg = 0;
        uint16_t count = 0;
        uint32_t period_ms = 0;
        ResultFn on_result = nullptr;  // called for every attempt, success or not
        void* ctx = nullptr;
        uint32_t next_due_ms = 0;  // owned by MBUS
    };

    // A single on-demand transaction. Trivially copyable on purpose, so another
    // task can hand it over through a queue.
    struct Request
    {
        enum class Kind : uint8_t
        {
            ReadRegs,
            WriteReg,
        };

        uint8_t slave = 0;  // 0 = broadcast write
        Kind kind = Kind::WriteReg;
        uint16_t reg = 0;
        uint16_t value_or_count = 0;
        uint8_t retries = 0;
        ResultFn on_done = nullptr;
        void* ctx = nullptr;
    };

    enum class DeviceState : uint8_t
    {
        Unknown,  // nothing heard yet
        Ok,
        Degraded,  // flaky: some traffic gets through, don't rely on it
        Lost,      // sustained silence
    };

    struct HealthPolicy
    {
        uint8_t degrade_after_consecutive = 2;
        uint8_t degrade_error_percent = 20;  // over the last 32 transactions
        uint8_t min_samples_for_rate = 16;   // don't judge a rate from a handful of samples
        uint8_t lost_after_consecutive = 10;
        uint8_t recover_after_good = 10;
        bool latch_degraded = false;  // stay Degraded until clear_latch()
    };

    struct DeviceStats
    {
        uint32_t total = 0;
        uint32_t ok = 0;
        uint32_t timeouts = 0;
        uint32_t crc_errors = 0;
        uint32_t bad_frames = 0;
        uint32_t transport_errors = 0;
        uint32_t exceptions = 0;  // replies that were exceptions; the link itself was fine
        uint8_t consecutive_failures = 0;
        uint8_t consecutive_good = 0;
        uint32_t history = 0;  // last 32 transactions, newest in bit 0, 1 = link failure
        uint32_t last_good_ms = 0;
        uint32_t last_latency_us = 0;
        uint32_t max_latency_us = 0;
        DeviceState state = DeviceState::Unknown;
        bool latched = false;

        uint8_t error_percent() const
        {
            const uint32_t n = total < 32 ? total : 32;
            if (n == 0)
                return 0;
            return static_cast<uint8_t>(static_cast<uint32_t>(__builtin_popcount(history)) * 100u / n);
        }
    };

    using StateChangeFn = void (*)(uint8_t slave, DeviceState from, DeviceState to, void* ctx);

    inline constexpr size_t kMaxJobsPerDevice = 2;

    struct Device
    {
        uint8_t slave = 0;
        PollJob jobs[kMaxJobsPerDevice];
        uint8_t job_count = 0;
        HealthPolicy policy;
        StateChangeFn on_state_change = nullptr;
        void* ctx = nullptr;
        DeviceStats stats;

        bool add_job(const PollJob& job)
        {
            if (job_count >= kMaxJobsPerDevice)
                return false;
            jobs[job_count++] = job;
            return true;
        }
    };

    // Owns one RS485 bus: polls registered devices, runs on-demand requests and
    // tracks each device's health. Fixed capacity, no heap. Every method must
    // be called from the one task that runs step(); other tasks should pass
    // Requests to it through a queue.
    template <size_t MaxDevices, size_t MaxRequests>
    class MBUS
    {
    public:
        explicit MBUS(rs485::Port& port, Clock& clock, uint32_t response_timeout_ms = 50)
            : clock_(clock),
              master_(port, clock, response_timeout_ms)
        {
        }

        bool add_device(const Device& device)
        {
            if (device_count_ >= MaxDevices || find_device(device.slave) != nullptr)
                return false;
            Device& d = devices_[device_count_++];
            d = device;
            const uint32_t now = clock_.now_ms();
            for (uint8_t i = 0; i < d.job_count; ++i)
                d.jobs[i].next_due_ms = now;
            return true;
        }

        bool submit(const Request& request)
        {
            if (queue_count_ >= MaxRequests)
                return false;
            queue_[(queue_head_ + queue_count_) % MaxRequests] = request;
            ++queue_count_;
            return true;
        }

        // Runs at most one transaction. Returns false if nothing was due.
        bool step()
        {
            const uint32_t now = clock_.now_ms();

            size_t dev_idx = 0;
            size_t job_idx = 0;
            const bool job_due = find_due_job(now, &dev_idx, &job_idx);

            // At most one on-demand request between polls, so a burst of them
            // can't starve the polling rate.
            if (queue_count_ > 0 && (!last_was_request_ || !job_due))
            {
                const Request request = queue_[queue_head_];
                queue_head_ = (queue_head_ + 1) % MaxRequests;
                --queue_count_;
                execute_request(request);
                last_was_request_ = true;
                return true;
            }

            if (job_due)
            {
                execute_job(devices_[dev_idx], devices_[dev_idx].jobs[job_idx]);
                last_was_request_ = false;
                return true;
            }
            return false;
        }

        // How long the caller may sleep before step() will have work.
        uint32_t ms_until_next_due()
        {
            if (queue_count_ > 0)
                return 0;
            const uint32_t now = clock_.now_ms();
            uint32_t best = UINT32_MAX;
            for (size_t i = 0; i < device_count_; ++i)
            {
                for (uint8_t j = 0; j < devices_[i].job_count; ++j)
                {
                    const PollJob& job = devices_[i].jobs[j];
                    if (job.period_ms == 0 || job.count == 0)
                        continue;
                    const int32_t remaining = static_cast<int32_t>(job.next_due_ms - now);
                    const uint32_t wait = remaining > 0 ? static_cast<uint32_t>(remaining) : 0;
                    if (wait < best)
                        best = wait;
                }
            }
            return best;
        }

        bool get_stats(uint8_t slave, DeviceStats* out) const
        {
            const Device* d = find_device(slave);
            if (d == nullptr)
                return false;
            *out = d->stats;
            return true;
        }

        // Lets a latched Degraded device return to Ok once it has recovered.
        bool clear_latch(uint8_t slave)
        {
            Device* d = find_device(slave);
            if (d == nullptr)
                return false;
            d->stats.latched = false;
            return true;
        }

    private:
        Device* find_device(uint8_t slave)
        {
            if (slave == kBroadcastAddress)
                return nullptr;
            for (size_t i = 0; i < device_count_; ++i)
                if (devices_[i].slave == slave)
                    return &devices_[i];
            return nullptr;
        }
        const Device* find_device(uint8_t slave) const
        {
            return const_cast<MBUS*>(this)->find_device(slave);
        }

        // Earliest deadline first: among due jobs, pick the one that has been due longest.
        bool find_due_job(uint32_t now, size_t* dev_out, size_t* job_out) const
        {
            bool found = false;
            uint32_t best_due = 0;
            for (size_t i = 0; i < device_count_; ++i)
            {
                for (uint8_t j = 0; j < devices_[i].job_count; ++j)
                {
                    const PollJob& job = devices_[i].jobs[j];
                    if (job.period_ms == 0 || job.count == 0)
                        continue;
                    if (static_cast<int32_t>(now - job.next_due_ms) < 0)
                        continue;
                    if (!found || static_cast<int32_t>(job.next_due_ms - best_due) < 0)
                    {
                        found = true;
                        best_due = job.next_due_ms;
                        *dev_out = i;
                        *job_out = j;
                    }
                }
            }
            return found;
        }

        void execute_job(Device& device, PollJob& job)
        {
            const Response r = master_.read_holding(device.slave, job.start_reg, job.count);
            record(device, r);

            job.next_due_ms += job.period_ms;
            const uint32_t after = clock_.now_ms();
            if (static_cast<int32_t>(after - job.next_due_ms) > 0)
                job.next_due_ms = after;  // fell behind: poll again straight away, but don't burst to catch up

            if (job.on_result)
                job.on_result(r, job.ctx);
        }

        void execute_request(const Request& request)
        {
            Response r;
            for (uint8_t attempt = 0;; ++attempt)
            {
                r = (request.kind == Request::Kind::ReadRegs)
                        ? master_.read_holding(request.slave, request.reg, request.value_or_count)
                        : master_.write_single(request.slave, request.reg, request.value_or_count);

                if (Device* d = find_device(request.slave))
                    record(*d, r);

                const bool final_result = r.result == Result::Ok || r.result == Result::ExceptionReply ||
                                          r.result == Result::InvalidRequest;
                if (final_result || attempt >= request.retries)
                    break;
            }
            if (request.on_done)
                request.on_done(r, request.ctx);
        }

        // An exception reply counts as a healthy link: the board answered, it
        // just refused (e.g. a register it can't currently supply).
        void record(Device& d, const Response& r)
        {
            if (r.result == Result::InvalidRequest)
                return;

            DeviceStats& s = d.stats;
            ++s.total;
            const bool link_ok = r.result == Result::Ok || r.result == Result::ExceptionReply;
            s.history = (s.history << 1) | (link_ok ? 0u : 1u);

            if (link_ok)
            {
                ++s.ok;
                if (r.result == Result::ExceptionReply)
                    ++s.exceptions;
                s.last_good_ms = r.timestamp_ms;
                s.last_latency_us = r.latency_us;
                if (r.latency_us > s.max_latency_us)
                    s.max_latency_us = r.latency_us;
                s.consecutive_failures = 0;
                if (s.consecutive_good < UINT8_MAX)
                    ++s.consecutive_good;
            }
            else
            {
                switch (r.result)
                {
                case Result::Timeout:
                    ++s.timeouts;
                    break;
                case Result::CrcError:
                    ++s.crc_errors;
                    break;
                case Result::BadFrame:
                    ++s.bad_frames;
                    break;
                default:
                    ++s.transport_errors;
                    break;
                }
                s.consecutive_good = 0;
                if (s.consecutive_failures < UINT8_MAX)
                    ++s.consecutive_failures;
            }

            update_state(d);
        }

        void update_state(Device& d)
        {
            DeviceStats& s = d.stats;
            const HealthPolicy& p = d.policy;
            const DeviceState old = s.state;
            DeviceState next = old;

            const bool rate_bad = s.total >= p.min_samples_for_rate && s.error_percent() >= p.degrade_error_percent;

            if (s.consecutive_failures >= p.lost_after_consecutive)
            {
                next = DeviceState::Lost;
            }
            else if (s.consecutive_failures >= p.degrade_after_consecutive || rate_bad)
            {
                next = DeviceState::Degraded;
            }
            else
            {
                switch (old)
                {
                case DeviceState::Unknown:
                    if (s.ok > 0)
                        next = DeviceState::Ok;
                    break;
                case DeviceState::Lost:
                    next = DeviceState::Degraded;  // link is back, but suspect until proven
                    break;
                case DeviceState::Degraded:
                    if (!s.latched && s.consecutive_good >= p.recover_after_good)
                        next = DeviceState::Ok;
                    break;
                case DeviceState::Ok:
                    break;
                }
            }

            if (next == DeviceState::Degraded && p.latch_degraded)
                s.latched = true;

            if (next != old)
            {
                s.state = next;
                if (d.on_state_change)
                    d.on_state_change(d.slave, old, next, d.ctx);
            }
        }

        Clock& clock_;
        Master master_;
        Device devices_[MaxDevices];
        size_t device_count_ = 0;
        Request queue_[MaxRequests];
        size_t queue_head_ = 0;
        size_t queue_count_ = 0;
        bool last_was_request_ = false;
    };
}  // namespace modbus
