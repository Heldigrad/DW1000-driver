#pragma once

#include "includes.h"
#include "spi_read_functions.h"
#include "spi_write_functions.h"

extern int idx;

void check();

void bip_init();

void bip_config();

void new_set_txfctrl(uint16_t txFrameLength);

void new_rx_enable(int mode);

void new_tx_start(int mode);

void rx_soft_reset(void);

void print_enabled_bits(uint32_t value);

void test_spi();