// usb_descriptors.c
//
// Composite device: CDC serial (stdio console) + CDC-NCM network adapter.
// Adapted from TinyUSB's examples/device/net_lwip_webserver (MIT).

#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"

// TinyUSB's test VID. TODO: request a Raspberry Pi PID before this leaves the bench.
#define USB_VID 0xCAFE
#define USB_PID 0x4D44

enum
{
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
    STRID_NET,
    STRID_MAC,
};

enum
{
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_NET,
    ITF_NUM_NET_DATA,
    ITF_NUM_TOTAL
};

#define EPNUM_CDC_NOTIF 0x81
#define EPNUM_CDC_OUT   0x02
#define EPNUM_CDC_IN    0x82
#define EPNUM_NET_NOTIF 0x83
#define EPNUM_NET_OUT   0x04
#define EPNUM_NET_IN    0x84

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0201,  // 2.01: has a BOS descriptor
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 1,
};

const uint8_t* tud_descriptor_device_cb(void) { return (const uint8_t*)&desc_device; }

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_CDC_NCM_DESC_LEN)

static const uint8_t desc_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, 64),
    TUD_CDC_NCM_DESCRIPTOR(ITF_NUM_NET, STRID_NET, STRID_MAC, EPNUM_NET_NOTIF, 64, EPNUM_NET_OUT, EPNUM_NET_IN,
                           CFG_TUD_NET_ENDPOINT_SIZE, CFG_TUD_NET_MTU),
};

const uint8_t* tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_configuration;
}

// Microsoft OS 2.0 descriptors: tell Windows to load its in-box NCM driver for
// the network function. Linux and macOS don't need them.
#define MS_OS_20_DESC_LEN        0xB2
#define BOS_TOTAL_LEN            (TUD_BOS_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define VENDOR_REQUEST_MICROSOFT 1

static const uint8_t desc_bos[] = {
    TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 1),
    TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT),
};

const uint8_t* tud_descriptor_bos_cb(void) { return desc_bos; }

static const uint8_t desc_ms_os_20[] = {
    // Set header: length, type, Windows version, total length
    U16_TO_U8S_LE(0x000A),
    U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR),
    U32_TO_U8S_LE(0x06030000),
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN),
    // Configuration subset header: length, type, configuration index, reserved, subset length
    U16_TO_U8S_LE(0x0008),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION),
    0,
    0,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A),
    // Function subset header: length, type, first interface, reserved, subset length
    U16_TO_U8S_LE(0x0008),
    U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION),
    ITF_NUM_NET,
    0,
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08),
    // Compatible ID: WINNCM
    U16_TO_U8S_LE(0x0014),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATIBLE_ID),
    'W',
    'I',
    'N',
    'N',
    'C',
    'M',
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
    // Registry property: DeviceInterfaceGUIDs = {12345678-0D08-43FD-8B3E-127CA8AFFF9D}
    U16_TO_U8S_LE(MS_OS_20_DESC_LEN - 0x0A - 0x08 - 0x08 - 0x14),
    U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
    U16_TO_U8S_LE(0x0007),
    U16_TO_U8S_LE(0x002A),
    'D',
    0x00,
    'e',
    0x00,
    'v',
    0x00,
    'i',
    0x00,
    'c',
    0x00,
    'e',
    0x00,
    'I',
    0x00,
    'n',
    0x00,
    't',
    0x00,
    'e',
    0x00,
    'r',
    0x00,
    'f',
    0x00,
    'a',
    0x00,
    'c',
    0x00,
    'e',
    0x00,
    'G',
    0x00,
    'U',
    0x00,
    'I',
    0x00,
    'D',
    0x00,
    's',
    0x00,
    0x00,
    0x00,
    U16_TO_U8S_LE(0x0050),
    '{',
    0x00,
    '1',
    0x00,
    '2',
    0x00,
    '3',
    0x00,
    '4',
    0x00,
    '5',
    0x00,
    '6',
    0x00,
    '7',
    0x00,
    '8',
    0x00,
    '-',
    0x00,
    '0',
    0x00,
    'D',
    0x00,
    '0',
    0x00,
    '8',
    0x00,
    '-',
    0x00,
    '4',
    0x00,
    '3',
    0x00,
    'F',
    0x00,
    'D',
    0x00,
    '-',
    0x00,
    '8',
    0x00,
    'B',
    0x00,
    '3',
    0x00,
    'E',
    0x00,
    '-',
    0x00,
    '1',
    0x00,
    '2',
    0x00,
    '7',
    0x00,
    'C',
    0x00,
    'A',
    0x00,
    '8',
    0x00,
    'A',
    0x00,
    'F',
    0x00,
    'F',
    0x00,
    'F',
    0x00,
    '9',
    0x00,
    'D',
    0x00,
    '}',
    0x00,
    0x00,
    0x00,
    0x00,
    0x00,
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, const tusb_control_request_t* request)
{
    if (stage != CONTROL_STAGE_SETUP)
        return true;
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR && request->bRequest == VENDOR_REQUEST_MICROSOFT &&
        request->wIndex == 7)
        return tud_control_xfer(rhport, request, (void*)(uintptr_t)desc_ms_os_20, sizeof(desc_ms_os_20));
    return false;  // stall anything else
}

static const char* const string_desc[] = {
    [STRID_MANUFACTURER] = "Perseus",
    [STRID_PRODUCT] = "Encoder Doom",
    [STRID_CDC] = "Encoder Doom console",
    [STRID_NET] = "Encoder Doom network",
};

static uint16_t desc_str[32 + 1];

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    size_t count = 0;
    char text[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

    if (index == STRID_LANGID)
    {
        desc_str[1] = 0x0409;  // English
        count = 1;
    }
    else
    {
        const char* str = NULL;
        if (index == STRID_SERIAL)
        {
            pico_get_unique_board_id_string(text, sizeof(text));
            str = text;
        }
        else if (index == STRID_MAC)
        {
            // NCM reads the MAC address as 12 hex digits.
            for (size_t i = 0; i < sizeof(tud_network_mac_address); i++)
            {
                text[2 * i] = "0123456789ABCDEF"[tud_network_mac_address[i] >> 4];
                text[2 * i + 1] = "0123456789ABCDEF"[tud_network_mac_address[i] & 0xF];
            }
            text[2 * sizeof(tud_network_mac_address)] = '\0';
            str = text;
        }
        else if (index < TU_ARRAY_SIZE(string_desc))
        {
            str = string_desc[index];
        }
        if (str == NULL)
            return NULL;

        count = strlen(str);
        if (count > 32)
            count = 32;
        for (size_t i = 0; i < count; i++)
            desc_str[1 + i] = (uint16_t)str[i];
    }

    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * count + 2));
    return desc_str;
}
