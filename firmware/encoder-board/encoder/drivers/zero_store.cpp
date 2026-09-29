// zero_store.cpp

#include "zero_store.hpp"

#include <cstddef>
#include <cstring>

#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "modbus/protocol.hpp"
#include "pico/flash.h"

namespace
{
    // Second-to-last sector: if the PICO_RP2350_A2_SUPPORTED CMake variable is ever
    // set, UF2s carry an absolute block at 0x10FFFF00 (RP2350-E10 workaround) and
    // every drag-and-drop flash would rewrite the last one.
    constexpr uint32_t kFlashOffset = PICO_FLASH_SIZE_BYTES - 2 * FLASH_SECTOR_SIZE;
    constexpr uint32_t kMagic = 0x5A45524Fu;  // "ZERO"
    constexpr uint32_t kErasedWord = 0xFFFFFFFFu;
    constexpr uint32_t kSafeExecuteTimeoutMs = 100;  // to park the other core, not for the erase itself

    struct Record
    {
        uint32_t magic;
        uint16_t offset;
        uint16_t crc;  // over magic and offset
    };

    uint16_t record_crc(const Record& record)
    {
        return modbus::crc16(reinterpret_cast<const uint8_t*>(&record), offsetof(Record, crc));
    }

    const Record& stored_record()
    {
        return *reinterpret_cast<const Record*>(XIP_BASE + kFlashOffset);
    }

    void erase_and_program(void* page)
    {
        flash_range_erase(kFlashOffset, FLASH_SECTOR_SIZE);
        flash_range_program(kFlashOffset, static_cast<const uint8_t*>(page), FLASH_PAGE_SIZE);
    }
}  // namespace

zero_store::LoadResult zero_store::load(uint16_t* offset)
{
    const Record record = stored_record();
    if (record.magic == kErasedWord)
        return LoadResult::kBlank;
    if (record.magic != kMagic || record.crc != record_crc(record))
        return LoadResult::kCorrupt;
    *offset = record.offset;
    return LoadResult::kOk;
}

bool zero_store::save(uint16_t offset)
{
    Record record{kMagic, offset, 0};
    record.crc = record_crc(record);

    uint8_t page[FLASH_PAGE_SIZE];
    std::memset(page, 0xFF, sizeof(page));
    std::memcpy(page, &record, sizeof(record));

    if (flash_safe_execute(&erase_and_program, page, kSafeExecuteTimeoutMs) != PICO_OK)
        return false;
    return std::memcmp(&stored_record(), &record, sizeof(record)) == 0;
}
