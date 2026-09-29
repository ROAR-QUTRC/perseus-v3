// net.cpp
//
// TinyUSB's NCM class carries Ethernet frames between the PC and lwIP.
// Adapted from TinyUSB's examples/device/net_lwip_webserver (MIT).

#include "net.hpp"

#include <cstring>

#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "pico/time.h"
#include "tusb.h"

extern "C"
{
#include "dhserver.h"

    // Read by the NCM driver and the MAC string descriptor. 0x02: locally administered.
    uint8_t tud_network_mac_address[6] = {0x02, 0x02, 0x84, 0x6A, 0x96, 0x00};
}

namespace
{
    constexpr ip4_addr_t ip4(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
    {
        return {PP_HTONL(LWIP_MAKEU32(a, b, c, d))};
    }

    const ip4_addr_t kAddress = ip4(192, 168, 7, 1);
    const ip4_addr_t kNetmask = ip4(255, 255, 255, 0);
    const ip4_addr_t kNone = ip4(0, 0, 0, 0);
    constexpr uint32_t kLeaseSeconds = 24 * 60 * 60;

    // No router or DNS server is advertised, so the PC keeps its own internet
    // route and name servers and only talks to the board on this link.
    dhcp_entry_t leases[] = {
        {{0}, ip4(192, 168, 7, 2), kLeaseSeconds},
        {{0}, ip4(192, 168, 7, 3), kLeaseSeconds},
    };
    const dhcp_config_t kDhcp = {kNone, 67, kNone, nullptr, TU_ARRAY_SIZE(leases), leases};

    netif interface;
    pbuf* received_frame = nullptr;  // set by tud_network_recv_cb(), consumed by poll()

    err_t link_output(netif*, pbuf* p)
    {
        for (;;)
        {
            if (!tud_ready())
                return ERR_USE;
            if (tud_network_can_xmit(p->tot_len))
            {
                tud_network_xmit(p, 0);
                return ERR_OK;
            }
            tud_task();  // let the previous transfer finish
        }
    }

    err_t ip4_output(netif* nif, pbuf* p, const ip4_addr_t* address) { return etharp_output(nif, p, address); }

    err_t interface_init(netif* nif)
    {
        nif->mtu = CFG_TUD_NET_MTU - SIZEOF_ETH_HDR;
        nif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP | NETIF_FLAG_UP;
        nif->name[0] = 'u';
        nif->name[1] = 's';
        nif->linkoutput = link_output;
        nif->output = ip4_output;
        return ERR_OK;
    }
}  // namespace

extern "C" bool tud_network_recv_cb(const uint8_t* src, uint16_t size)
{
    if (received_frame)
        return false;  // previous frame not handed to lwIP yet; the driver retries
    if (size)
    {
        pbuf* p = pbuf_alloc(PBUF_RAW, size, PBUF_POOL);
        if (p)
        {
            pbuf_take(p, src, size);
            received_frame = p;
        }
    }
    return true;
}

extern "C" uint16_t tud_network_xmit_cb(uint8_t* dst, void* ref, uint16_t)
{
    auto* p = static_cast<pbuf*>(ref);
    return pbuf_copy_partial(p, dst, p->tot_len, 0);
}

extern "C" void tud_network_init_cb(void)
{
    if (received_frame)
    {
        pbuf_free(received_frame);
        received_frame = nullptr;
    }
}

extern "C" u32_t sys_now(void) { return to_ms_since_boot(get_absolute_time()); }

void net::init()
{
    lwip_init();

    // lwIP's MAC must differ from the one the PC sees for its end of the link.
    interface.hwaddr_len = sizeof(tud_network_mac_address);
    std::memcpy(interface.hwaddr, tud_network_mac_address, sizeof(tud_network_mac_address));
    interface.hwaddr[5] ^= 0x01;

    netif_add(&interface, &kAddress, &kNetmask, &kNone, nullptr, interface_init, ethernet_input);
    netif_set_default(&interface);

    while (dhserv_init(&kDhcp) != ERR_OK)
    {
    }
}

void net::poll()
{
    if (received_frame)
    {
        // ethernet_input() takes ownership unless it fails.
        if (ethernet_input(received_frame, &interface) != ERR_OK)
            pbuf_free(received_frame);
        received_frame = nullptr;
        tud_network_recv_renew();
    }
    sys_check_timeouts();
}
