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

void set_rx_timeout(uint16_t time);

int32_t read_carrier_integrator(void);

uint64_t send_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id);

uint64_t send_resp1_message(uint8_t src_id, uint64_t T2, uint8_t message_id);

uint64_t send_poll2_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id);

int send_resp2_message(uint8_t src_id, uint64_t T3, uint64_t T6, uint8_t message_id);

int get_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp);

int get_resp1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T2);

int get_poll2_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp);

int get_resp2_message(uint8_t src_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T3, uint64_t *T6);

void get_msg_from_init(uint8_t my_id, uint64_t *T2, uint64_t *T3, uint64_t *T6, uint8_t *message_id);

int get_msg_from_resp(uint8_t destination_id, uint64_t *T2, uint64_t *T3, uint64_t *T4, uint64_t *T5, uint64_t *T6, uint8_t message_id);

uint8_t get_rx_frame_len();

uint64_t dw1000_read_sys_time(void);

bool has_1_second_passed(uint64_t start_time, uint64_t current_time);

void set_antenna_delay(int anchor_id);

double compute_ds_twr_distance_basic(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4, uint64_t T5, uint64_t T6);

double get_coord(double B1, double B2, double DO, double D1, double D2);

void compute_coord(double Distances[4]);