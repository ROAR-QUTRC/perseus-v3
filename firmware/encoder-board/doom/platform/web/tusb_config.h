// tusb_config.h
//
// TinyUSB for the web backend: CDC (stdio console) plus NCM (USB network link).

#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_PICO
#endif

#define CFG_TUSB_RHPORT0_MODE  OPT_MODE_DEVICE
#define CFG_TUD_ENABLED        1
#define CFG_TUD_ENDPOINT0_SIZE 64

#define CFG_TUD_CDC            1
#define CFG_TUD_CDC_RX_BUFSIZE 64
#define CFG_TUD_CDC_TX_BUFSIZE 1024

#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_NCM       1
// Two transfer blocks each way, so one can fill while the other is on the wire.
#define CFG_TUD_NCM_IN_NTB_N  2
#define CFG_TUD_NCM_OUT_NTB_N 2

#endif
