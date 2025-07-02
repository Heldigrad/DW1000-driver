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

uint64_t send_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id);

uint64_t send_resp1_message(uint8_t src_id, uint64_t T2, uint8_t message_id);

int send_resp2_message(uint8_t src_id, uint64_t T3, uint64_t T6, uint8_t message_id);

int get_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp);

int get_resp1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T2);

int get_poll2_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp);

int get_resp2_message(uint8_t src_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T3, uint64_t *T6);

void get_msg_from_init(uint8_t my_id, uint64_t *T2, uint64_t *T3, uint64_t *T6, uint8_t *message_id);

int get_msg_from_resp(uint8_t destination_id, uint64_t *T2, uint64_t *T3, uint64_t *T4, uint64_t *T5, uint64_t *T6, uint8_t message_id);

double compute_distance_meters(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4);

int compare(const void *a, const void *b);

double mean_distance(double distances[NR_OF_DISTANCES]);

uint8_t get_rx_frame_len();

int send_distance(uint8_t src_id, uint32_t distance);

uint64_t dw1000_read_sys_time(void);

bool has_1_second_passed(uint64_t start_time, uint64_t current_time);

void set_antenna_delay(int anchor_id);

int32_t read_carrier_integrator(void);

double compute_ds_twr_distance_basic(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4, uint64_t T5, uint64_t T6);

double compute_ds_twr_distance_with_cor(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4, uint64_t T5, uint64_t T6, double ppm_offset);

double compute_ds_twr_distance(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4, uint64_t T5, uint64_t T6);

int32_t read_carrier_integrator_gipi(void);