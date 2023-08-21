#ifndef GLOBAL_H
#define GLOBAL_H

#include "comms/radio.h"

extern RFM97_LoRa_config radioconfig;
extern bool radioconfig_updated;

void telem_rx_cb();

#endif