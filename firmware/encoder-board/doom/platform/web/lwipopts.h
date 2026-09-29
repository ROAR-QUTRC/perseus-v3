// lwipopts.h
//
// lwIP for the web backend: no RTOS, polled from core 1's loop, TCP tuned so
// about one compressed frame can be queued at once.

#ifndef LWIPOPTS_H
#define LWIPOPTS_H

#define NO_SYS 1
#define SYS_LIGHTWEIGHT_PROT 0  // lwIP only ever runs on core 1
#define LWIP_RAW 0
#define LWIP_NETCONN 0
#define LWIP_SOCKET 0
#define LWIP_STATS 0

#define LWIP_IPV4 1
#define LWIP_IPV6 0
#define LWIP_ICMP 1
#define LWIP_UDP 1
#define LWIP_TCP 1
#define LWIP_DHCP 0  // we are the DHCP server (dhserver.c), not a client
#define LWIP_SINGLE_NETIF 1
#define ETH_PAD_SIZE 0
// The DHCP server has to see the PC's requests, which arrive before it has an address.
#define LWIP_IP_ACCEPT_UDP_PORT(p) ((p) == PP_NTOHS(67))

#define MEM_ALIGNMENT 4
#define MEM_SIZE (40 * 1024)  // holds the whole TCP send buffer: frame data is copied in
#define PBUF_POOL_SIZE 16
#define MEMP_NUM_PBUF 16       // the page body is sent straight from flash
#define MEMP_NUM_TCP_PCB 8     // stream, page, and a few key POSTs at once
#define MEMP_NUM_TCP_SEG 64

#define TCP_MSS 1460
#define TCP_SND_BUF (16 * TCP_MSS)
#define TCP_SND_QUEUELEN 64
#define TCP_WND (4 * TCP_MSS)

#endif
