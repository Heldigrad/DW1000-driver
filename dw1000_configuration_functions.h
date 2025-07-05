#pragma once

#include "includes.h"
#include "spi_read_functions.h"
#include "spi_write_functions.h"

void initialize();

void configure();

void set_txfctrl(uint16_t txFrameLength);

void rx_enable(int mode);

void tx_start(int mode);

void rx_soft_reset(void);

void print_enabled_bits(uint32_t value);

void test_spi();