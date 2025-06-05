#pragma once

#include "includes.h"
#include "spi_read_functions.h"
#include "spi_write_functions.h"
#include "dw1000_configuration_functions.h"

void set_rx_antenna_delay(uint16_t rxDelay);

void set_tx_antenna_delay(uint16_t txDelay);

uint64_t get_tx_timestamp();

uint64_t get_rx_timestamp();

int receive(uint64_t *buffer, uint64_t *timestamp);

int transmit(uint64_t data, int len, uint64_t *timestamp);

double compute_distance(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4);

void set_rx_after_tx_delay(uint32_t rxDelayTime);

void set_rx_timeout(uint16_t time);

void set_delayed_trx_time(uint32_t starttime);