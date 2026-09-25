#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Holds a FreeRTOS mutex for the lifetime of the object.
class Lock
{
public:
    explicit Lock(SemaphoreHandle_t mutex)
        : _mutex(mutex)
    {
        xSemaphoreTake(_mutex, portMAX_DELAY);
    }
    ~Lock() { xSemaphoreGive(_mutex); }

    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;

private:
    SemaphoreHandle_t _mutex;
};
