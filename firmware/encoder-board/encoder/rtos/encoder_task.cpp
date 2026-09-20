// encoder_task.cpp

#include "encoder_task.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "as5600.hpp"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "shared_state.hpp"
#include "task.h"

// GPIOs wired to the AS5600's SDA/SCL lines. Overridable at build time, e.g.:
//   target_compile_definitions(encoder PRIVATE AS5600_SDA_PIN=2 AS5600_SCL_PIN=3)
#ifndef AS5600_SDA_PIN
#define AS5600_SDA_PIN 2
#endif

#ifndef AS5600_SCL_PIN
#define AS5600_SCL_PIN 3
#endif

namespace
{
    // GPIO2/3 fall on the I2C1 pin pair, not I2C0
    i2c_inst_t* const kI2cPort = i2c1;
    constexpr uint kI2cBaudHz = 400000;  // AS5600 supports Fast mode, up to 1MHz

    // Angle sampling/publishing runs at this rate -- 1kHz. I2C at 400kHz
    // comfortably fits a magnet_detected() + read() cycle inside 1ms.
    constexpr uint32_t kSamplePeriodMs = 1;

    // Velocity direction is deliberately NOT computed from consecutive
    // 1ms samples: a real rotation that produces a clear multi-count
    // delta over 50ms often produces a sub-1-count delta over 1ms --
    // below the AS5600's resolution, so an instantaneous 1ms delta would
    // just be noise most of the time. Instead, the reference sample used
    // for the delta only advances every kVelocityWindowSamples samples,
    // giving the same effective ~50ms window (and therefore the same
    // deadband tuning) as before this was 1kHz, while every single 1ms
    // sample still gets read, published, and available to comms_task.
    constexpr int kVelocityWindowSamples = 50;  // 50 samples @ 1kHz = 50ms

    // Ignore raw-count deltas at or below this across one velocity
    // window; the AS5600's last bit or two of noise would otherwise make
    // the status LED flicker between velocity states while the magnet is
    // actually still.
    constexpr int32_t kVelocityDeadbandCounts = 2;

    // Only printed every this many samples (@ 1kHz, 200 samples = 200ms,
    // ~5Hz) -- printing every 1ms sample would flood the terminal and
    // risk stdio_usb backpressure blowing the 1kHz timing budget if
    // nothing's reading the port.
    constexpr int kPrintEverySamples = 200;

    int velocity_sign(uint16_t previous_counts, uint16_t current_counts)
    {
        int32_t delta = static_cast<int32_t>(current_counts) - static_cast<int32_t>(previous_counts);
        if (delta > 2048)
            delta -= 4096;  // wrapped backward through 0
        else if (delta < -2048)
            delta += 4096;  // wrapped forward through 4095

        if (delta > kVelocityDeadbandCounts)
            return 1;
        if (delta < -kVelocityDeadbandCounts)
            return -1;
        return 0;
    }
}  // namespace

void encoder_task(void* parameter)
{
    auto* shared = static_cast<SharedState*>(parameter);

    i2c_init(kI2cPort, kI2cBaudHz);
    gpio_set_function(AS5600_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(AS5600_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(AS5600_SDA_PIN);
    gpio_pull_up(AS5600_SCL_PIN);

    As5600 encoder(kI2cPort);

    bool have_reference = false;
    uint16_t reference_counts = 0;
    int samples_since_reference = 0;
    int current_velocity_sign = 0;

    int print_counter = 0;

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // Drain any pending commands before this cycle's read, so a zero
        // request takes effect on the very next reading rather than a
        // stale one.
        EncoderCommand cmd;
        while (xQueueReceive(shared->encoder_commands, &cmd, 0) == pdTRUE)
        {
            if (cmd == EncoderCommand::kZero)
                encoder.zero();
        }

        EncoderSample sample;
        sample.magnet_detected = encoder.magnet_detected();

        if (sample.magnet_detected)
        {
            sample.valid = encoder.read(&sample.reading);
            if (sample.valid)
            {
                if (!have_reference)
                {
                    reference_counts = sample.reading.raw_counts;
                    have_reference = true;
                    samples_since_reference = 0;
                }
                else if (++samples_since_reference >= kVelocityWindowSamples)
                {
                    current_velocity_sign = velocity_sign(reference_counts, sample.reading.raw_counts);
                    reference_counts = sample.reading.raw_counts;
                    samples_since_reference = 0;
                }

                if (++print_counter >= kPrintEverySamples)
                {
                    printf("angle: %4u (%.1f deg)\n", sample.reading.raw_counts, sample.reading.degrees);
                    print_counter = 0;
                }
            }
            else
            {
                printf("AS5600: I2C read failed\n");
                have_reference = false;
                current_velocity_sign = 0;
            }
        }
        else
        {
            if (++print_counter >= kPrintEverySamples)
            {
                printf("AS5600: no magnet detected\n");
                print_counter = 0;
            }
            have_reference = false;
            current_velocity_sign = 0;
        }

        sample.velocity_sign = current_velocity_sign;
        xQueueOverwrite(shared->latest_sample, &sample);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(kSamplePeriodMs));
    }
}
