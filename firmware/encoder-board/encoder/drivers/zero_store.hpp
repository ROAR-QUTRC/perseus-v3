// zero_store.hpp
//
// Keeps the AS5600 zero offset in flash so it survives a power cycle.

#pragma once

#include <cstdint>

namespace zero_store
{
    enum class LoadResult
    {
        kOk,
        kBlank,    // never saved
        kCorrupt,  // bad magic or CRC, e.g. power lost mid-save
    };

    // Reads through XIP, so it is safe to call from anywhere.
    LoadResult load(uint16_t* offset);

    // Erases and rewrites the record, then reads it back. Stalls both cores for
    // the sector erase (typically 45 ms, up to 400 ms), so only call it on a zero.
    bool save(uint16_t offset);
}  // namespace zero_store
