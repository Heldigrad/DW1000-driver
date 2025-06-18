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

int32_t read_carrier_integrator(void);

double compute_distance_rep(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4);

uint64_t send_poll_message(uint8_t src_id, uint8_t dest_id);

int get_poll_message(uint8_t src_id, uint8_t dest_id, uint64_t *timestamp);

uint64_t send_resp_message(uint8_t src_id, uint8_t dest_id);

int get_resp_message(uint8_t src_id, uint8_t my_id, uint64_t *timestamp);

int send_timestamps(uint8_t msg_id, uint64_t T1, uint64_t T4);

int get_timestamps(uint8_t msg_id, uint64_t *T1, uint64_t *T4);

int get_msg_from_init(uint64_t *T1, uint64_t *T2, uint64_t *T3, uint64_t *T4);

double compute_distance_meters(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4);

int compare(const void *a, const void *b);

double mean_distance(double distances[NR_OF_DISTANCES]);

uint8_t get_rx_frame_len();