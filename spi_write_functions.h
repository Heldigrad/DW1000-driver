#pragma once

#include "includes.h"

// SPI write
int dw1000_write(uint8_t reg, uint8_t *data, size_t len);

int dw1000_write_u8(uint8_t reg, uint8_t value);

int dw1000_write_u16(uint8_t reg, uint16_t value);

int dw1000_write_u32(uint8_t reg, uint32_t value);

int dw1000_write_u64(uint8_t reg, uint64_t value);

// SPI write sub-address
int dw1000_subwrite(uint8_t reg, uint16_t subaddr, uint8_t *data, size_t len);

int dw1000_subwrite_u8(uint8_t reg, uint16_t subaddr, uint8_t value);

int dw1000_subwrite_u16(uint8_t reg, uint16_t subaddr, uint16_t value);

int dw1000_subwrite_u32(uint8_t reg, uint16_t subaddr, uint32_t value);

int dw1000_subwrite_u40(uint8_t reg, uint16_t subaddr, uint64_t value);

int dw1000_subwrite_u64(uint8_t reg, uint16_t subaddr, uint64_t value);
