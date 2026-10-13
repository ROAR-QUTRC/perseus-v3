// encoder_task.cpp

#include "encoder_task.hpp"

#include <cstdio>

#include "FreeRTOS.h"
#include "as5600.hpp"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "shared_state.hpp"
#include "task.h"

// AS5600 SDA/SCL pins, overridable at build time.
#ifndef AS5600_SDA_PIN
#define AS5600_SDA_PIN 2
#endif

#ifndef AS5600_SCL_PIN
#define AS5600_SCL_PIN 3
#endif

namespace
{
    // GPIO2/3 are the I2C1 pin pair.
    i2c_inst_t* const kI2cPort = i2c1;
    constexpr uint kI2cBaudHz = 400000;  // AS5600 supports Fast mode, up to 1MHz

    // 1kHz; a magnet_detected() + read() cycle at 400kHz I2C fits inside 1ms.
    constexpr uint32_t kSamplePeriodMs = 1;

    // Velocity direction is taken over a 50ms window, not between consecutive
    // 1ms samples: real motion is often under one count per ms, which is
    // noise at the AS5600's resolution. Every 1ms sample is still published.
    constexpr int kVelocityWindowSamples = 50;  // 50 samples @ 1kHz = 50ms

    // Deltas at or below this are noise; without it the LED flickers while the magnet is still.
    constexpr int32_t kVelocityDeadbandCounts = 2;

    // Print at ~5Hz: printing every sample would risk stdio_usb backpressure breaking the 1kHz timing.
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
        // Drain commands first so a zero takes effect on this cycle's reading.
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
                // TODO: rate-limit like the other prints; a failing sensor prints every
                // 1 ms and USB backpressure can break the 1 kHz loop.
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
