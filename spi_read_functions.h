#pragma once
#include "includes.h"

// SPI read registers

int dw1000_read(uint8_t reg, uint8_t *data, size_t len);

int dw1000_read_u8(uint8_t reg, uint8_t *value);

int dw1000_read_u16(uint8_t reg, uint16_t *value);

int dw1000_read_u32(uint8_t reg, uint32_t *value);

int dw1000_read_u64(uint8_t reg, uint64_t *value);

// SPI read register sub-addresses

int dw1000_subread(uint8_t reg, uint16_t subaddr, uint8_t *data, size_t len);

int dw1000_subread_u8(uint8_t reg, uint16_t subaddr, uint8_t *value);

int dw1000_subread_u16(uint8_t reg, uint16_t subaddr, uint16_t *value);

int dw1000_subread_u32(uint8_t reg, uint16_t subaddr, uint32_t *value);

int dw1000_subread_u40(uint8_t reg, uint8_t subreg, uint64_t *value);
