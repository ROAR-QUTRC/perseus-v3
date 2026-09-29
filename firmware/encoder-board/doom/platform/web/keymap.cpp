// keymap.cpp

#include "keymap.hpp"

#include <cstring>

#include "class/hid/hid.h"

namespace
{
    struct Entry
    {
        const char* code;
        uint8_t usage;
    };

    // Letters, digits and F-keys follow a pattern and are handled in code.
    constexpr Entry kNamed[] = {
        {"ArrowUp", HID_KEY_ARROW_UP},
        {"ArrowDown", HID_KEY_ARROW_DOWN},
        {"ArrowLeft", HID_KEY_ARROW_LEFT},
        {"ArrowRight", HID_KEY_ARROW_RIGHT},
        {"Enter", HID_KEY_ENTER},
        {"Escape", HID_KEY_ESCAPE},
        {"Backspace", HID_KEY_BACKSPACE},
        {"Tab", HID_KEY_TAB},
        {"Space", HID_KEY_SPACE},
        {"Minus", HID_KEY_MINUS},
        {"Equal", HID_KEY_EQUAL},
        {"BracketLeft", HID_KEY_BRACKET_LEFT},
        {"BracketRight", HID_KEY_BRACKET_RIGHT},
        {"Backslash", HID_KEY_BACKSLASH},
        {"Semicolon", HID_KEY_SEMICOLON},
        {"Quote", HID_KEY_APOSTROPHE},
        {"Backquote", HID_KEY_GRAVE},
        {"Comma", HID_KEY_COMMA},
        {"Period", HID_KEY_PERIOD},
        {"Slash", HID_KEY_SLASH},
        {"Pause", HID_KEY_PAUSE},
        {"Insert", HID_KEY_INSERT},
        {"Home", HID_KEY_HOME},
        {"PageUp", HID_KEY_PAGE_UP},
        {"Delete", HID_KEY_DELETE},
        {"End", HID_KEY_END},
        {"PageDown", HID_KEY_PAGE_DOWN},
        {"ControlLeft", HID_KEY_CONTROL_LEFT},
        {"ShiftLeft", HID_KEY_SHIFT_LEFT},
        {"AltLeft", HID_KEY_ALT_LEFT},
        {"ControlRight", HID_KEY_CONTROL_RIGHT},
        {"ShiftRight", HID_KEY_SHIFT_RIGHT},
        {"AltRight", HID_KEY_ALT_RIGHT},
    };

    bool equals(const char* code, size_t length, const char* name)
    {
        return std::strlen(name) == length && std::memcmp(code, name, length) == 0;
    }

    bool has_prefix(const char* code, size_t length, const char* prefix)
    {
        const size_t n = std::strlen(prefix);
        return length > n && std::memcmp(code, prefix, n) == 0;
    }
}  // namespace

uint8_t keymap::usage_from_code(const char* code, size_t length)
{
    if (length == 4 && has_prefix(code, length, "Key") && code[3] >= 'A' && code[3] <= 'Z')
        return static_cast<uint8_t>(HID_KEY_A + (code[3] - 'A'));

    if (length == 6 && has_prefix(code, length, "Digit") && code[5] >= '0' && code[5] <= '9')
        return code[5] == '0' ? HID_KEY_0 : static_cast<uint8_t>(HID_KEY_1 + (code[5] - '1'));

    if (has_prefix(code, length, "F") && length <= 3)
    {
        int n = 0;
        for (size_t i = 1; i < length; ++i)
        {
            if (code[i] < '0' || code[i] > '9')
                return 0;
            n = n * 10 + (code[i] - '0');
        }
        return (n >= 1 && n <= 12) ? static_cast<uint8_t>(HID_KEY_F1 + (n - 1)) : 0;
    }

    for (const Entry& entry : kNamed)
        if (equals(code, length, entry.code))
            return entry.usage;
    return 0;
}
