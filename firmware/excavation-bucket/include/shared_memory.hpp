/**
 * @brief The EnC board's shared memory structure and utilities.
 * @details This file defines the shared memory structs that allow the canbus core and controlling core to read and set independently.
 */

#pragma once

#include "encoder_bus.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "hi_can_parameter.hpp"

/**
 * @brief Reusable RAII Lock guard for FreeRTOS semaphores/mutexes.
 * @details Automatically takes the mutex on construction and releases it on destruction.
 */

// TODO: this is a copy of Mozz's lock, consider refactoring or using a common implementation if available.
class Lock
{
public:
    explicit Lock(SemaphoreHandle_t mutex)
        : _mutex(mutex)
    {
        if (_mutex != nullptr)
        {
            xSemaphoreTake(_mutex, portMAX_DELAY);
        }
    }

    ~Lock()
    {
        if (_mutex != nullptr)
        {
            xSemaphoreGive(_mutex);
        }
    }

    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;

private:
    SemaphoreHandle_t _mutex;
};

/**
 * @brief Abstract class to represent a generic excavation joint in shared memory.
 */
class ExcavationJoint
{
public:
    virtual ~ExcavationJoint() = default;

    virtual void set_speed(int16_t value) = 0;
    virtual void set_target_position(int16_t value) = 0;
    virtual int16_t get_current_position() const = 0;
    virtual void monitor_and_move(void) = 0;
};

/**
 * @brief The shared memory for a single motor/joint.
 * @details This class provides thread-safe access to the motor's speed, target position, and current position.
 */
class MotorMemory : public ExcavationJoint
{
public:
    MotorMemory(EncoderId encoder_id, uint8_t encoder_group_id, EncoderBus* encoder_bus)
        : _encoder_id(encoder_id),
          _encoder_group_id(encoder_group_id),
          _encoder_bus(encoder_bus)
    {
        _mutex = xSemaphoreCreateMutex();
    }

    ~MotorMemory() override
    {
        if (_mutex != nullptr)
        {
            vSemaphoreDelete(_mutex);
        }
    }

    EncoderId encoder_id() const { return _encoder_id; }
    uint8_t encoder_group_id() const { return _encoder_group_id; }

    int16_t get_speed() const
    {
        Lock lock(_mutex);
        return _speed;
    }

    void set_speed(int16_t value) override
    {
        Lock lock(_mutex);
        _speed = value;
    }

    int16_t get_target_position() const
    {
        Lock lock(_mutex);
        return _target_position;
    }

    void set_target_position(int16_t value) override
    {
        Lock lock(_mutex);
        _target_position = value;
    }

    int16_t get_current_position() const override
    {
        Lock lock(_mutex);
        return _current_position;
    }

    void set_current_position(EncoderReading reading)
    {
        if (reading.angle_valid)
        {
            Lock lock(_mutex);
            _current_position = static_cast<int16_t>(reading.degrees);
        }
    }

    void monitor_and_move(void) override
    {
        EncoderReading reading;
        _encoder_bus->get(_encoder_id, &reading);
        set_current_position(reading);
    }

private:
    const EncoderId _encoder_id;
    const uint8_t _encoder_group_id;
    EncoderBus* _encoder_bus;

    int16_t _target_position = 0;
    int16_t _current_position = 0;
    int16_t _speed = 0;

    SemaphoreHandle_t _mutex = nullptr;
};