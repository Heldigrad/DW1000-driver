#pragma once

#include "includes.h"

int dw1000_read(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[1];
    tx_buf[0] = reg & 0x3F; // Read operation: MSB=0
    // tx_buf[1] = 0;

    struct spi_buf tx_bufs[] = {
        {.buf = &tx_buf, .len = 1}, // Send register address
    };
    struct spi_buf rx_bufs[] = {
        {.buf = NULL, .len = 1},
        {.buf = data, .len = len}, // Receive data
    };

    struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
    struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};

    gpio_pin_set_dt(&cs_gpio, 0); // Assert CS
    k_msleep(1);
    int ret = spi_transceive_dt(&spispec, &tx, &rx);
    k_msleep(1);
    gpio_pin_set_dt(&cs_gpio, 1); // Deassert CS

    if (ret)
    {
        LOG_ERR("SPI read failed: %d", ret);
        return ret;
    }
    /*
        // Log the received data
        LOG_INF("SPI read successful. Data from register 0x%X: ", reg);
        for (size_t i = 0; i < len; i++)
        {
            LOG_INF("Byte %zu: 0x%02X", i, data[i]);
        }
    */
    return 0;
}

int dw1000_read_u8(uint8_t reg, uint8_t *value)
{
    uint8_t buffer[1];
    int ret = dw1000_read(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to read 16-bit value from register 0x%X", reg);
        return ret;
    }

    *value = buffer[0];
    LOG_INF("8-bit read from 0x%X: 0x%04X", reg, *value);
    return 0;
}

int dw1000_read_u16(uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];
    int ret = dw1000_read(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to read 16-bit value from register 0x%X", reg);
        return ret;
    }

    *value = ((uint16_t)buffer[1] << 8) | buffer[0];
    LOG_INF("16-bit read from 0x%X: 0x%04X", reg, *value);
    return 0;
}

int dw1000_read_u32(uint8_t reg, uint32_t *value)
{
    uint8_t buffer[4];
    int ret = dw1000_read(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to read 32-bit value from register 0x%X", reg);
        return ret;
    }

    *value = ((uint32_t)buffer[3] << 24) |
             ((uint32_t)buffer[2] << 16) |
             ((uint32_t)buffer[1] << 8) |
             buffer[0];

    LOG_INF("32-bit read from 0x%X: 0x%08X", reg, *value);
    return 0;
}

// SPI read sub-addresses

int dw1000_subread(uint8_t reg, uint16_t subaddr, uint8_t *data, size_t len)
{
    uint8_t header[3];
    size_t header_len = 0;

    if (subaddr <= 0x7F)
    {
        // Short sub-index format: 2-byte header
        header[0] = 0x40 | reg;     // Bit 6 = 1 (subindex), Bit 7 = 0 (read)
        header[1] = subaddr & 0x7F; // MSB is 0 (short format)
        header_len = 2;
    }
    else
    {
        // Long sub-index format: 3-byte header
        header[0] = 0x40 | reg;              // Bit 6 = 1 (subindex), Bit 7 = 0 (read)
        header[1] = 0x80 | (subaddr & 0x7F); // Bit 7 = 1 (long format), bits 6-0 = low 7 bits
        header[2] = (subaddr >> 7) & 0xFF;   // MSB of subaddress
        header_len = 3;
    }

    struct spi_buf tx_bufs[] = {
        {.buf = header, .len = header_len},
    };
    struct spi_buf rx_bufs[] = {
        {.buf = NULL, .len = header_len},
        {.buf = data, .len = len},
    };

    struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};
    struct spi_buf_set rx = {.buffers = rx_bufs, .count = 2};

    gpio_pin_set_dt(&cs_gpio, 0); // Assert CS
    int ret = spi_transceive_dt(&spispec, &tx, &rx);
    gpio_pin_set_dt(&cs_gpio, 1); // Deassert CS

    if (ret)
    {
        LOG_ERR("SPI sub-read failed: reg=0x%02X, sub=0x%04X, err=%d", reg, subaddr, ret);
    }
    // else{
    //     LOG_INF("Read data from reg %X : %X:", reg, subaddr);
    //     for(int i = 0; i< len; ++i){
    //         LOG_INF("Byte[%d] = %X", i, data[i]);
    //     }
    // }

    return ret;
}

int dw1000_subread_u8(uint8_t reg, uint16_t subaddr, uint8_t *value)
{
    return dw1000_subread(reg, subaddr, value, sizeof(uint8_t));
}

int dw1000_subread_u16(uint8_t reg, uint16_t subaddr, uint16_t *value)
{
    uint8_t buffer[2];
    int ret = dw1000_subread(reg, subaddr, buffer, sizeof(buffer));
    if (ret == 0)
    {
        *value = buffer[0] | ((uint16_t)buffer[1] << 8);
    }
    return ret;
}

int dw1000_subread_u32(uint8_t reg, uint16_t subaddr, uint32_t *value)
{
    uint8_t buffer[4];
    int ret = dw1000_subread(reg, subaddr, buffer, sizeof(buffer));
    if (ret == 0)
    {
        *value = buffer[0] |
                 ((uint32_t)buffer[1] << 8) |
                 ((uint32_t)buffer[2] << 16) |
                 ((uint32_t)buffer[3] << 24);
    }
    return ret;
}

int dw1000_subread_u40(uint8_t reg, uint8_t subreg, uint64_t *value)
{
    uint8_t buffer[5];
    int ret = dw1000_subread(reg, subreg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        return ret;
    }

    *value = (uint64_t)buffer[0] |
             ((uint64_t)buffer[1] << 8) |
             ((uint64_t)buffer[2] << 16) |
             ((uint64_t)buffer[3] << 24) |
             ((uint64_t)buffer[4] << 32);

    return 0;
}
