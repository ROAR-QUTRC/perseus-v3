// net.hpp
//
// USB network link: the board is 192.168.7.1 and hands the PC 192.168.7.2 by DHCP.

#pragma once

namespace net
{
    // After tusb_init(). Brings up lwIP and the DHCP server.
    void init();

    // Every loop pass: feeds received frames to lwIP and runs its timers.
    void poll();
}  // namespace net
