#include "spi_write_functions.h"
#include "devices.h"

// SPI write
int dw1000_write(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t tx_buf[5];             // Register header + data
    tx_buf[0] = 0x80 | reg;        // Op. bit + address
    memcpy(&tx_buf[1], data, len); // Data to be written

    struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = len + 1},
    };

    struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};

    gpio_pin_set_dt(&cs_gpio, 0); // Assert CS
    int ret = spi_write_dt(&spispec, &tx);
    gpio_pin_set_dt(&cs_gpio, 1); // Deassert CS

    if (ret)
    {
        LOG_ERR("SPI write failed: %d", ret);
    }
    return ret;
}

int dw1000_write_u8(uint8_t reg, uint8_t value)
{
    uint8_t buffer[1];
    buffer[0] = value;

    int ret = dw1000_write(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to write 8-bit value 0x%02X to register 0x%X", value, reg);
    }
    else
    {
        // LOG_INF("Wrote 8-bit value 0x%02X to register 0x%X", value, reg);
    }
    return ret;
}

int dw1000_write_u16(uint8_t reg, uint16_t value)
{
    uint8_t buffer[2];
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);

    int ret = dw1000_write(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to write 16-bit value 0x%04X to register 0x%X", value, reg);
    }
    else
    {
        // LOG_INF("Wrote 16-bit value 0x%04X to register 0x%X", value, reg);
    }
    return ret;
}

int dw1000_write_u32(uint8_t reg, uint32_t value)
{
    uint8_t buffer[4];
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[3] = (uint8_t)((value >> 24) & 0xFF);

    int ret = dw1000_write(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to write 32-bit value 0x%08X to register 0x%X", value, reg);
    }
    else
    {
        //  LOG_INF("Wrote 32-bit value 0x%08X to register 0x%X", value, reg);
    }
    return ret;
}

int dw1000_write_u64(uint8_t reg, uint64_t value)
{
    uint8_t buffer[8];
    buffer[0] = (uint8_t)(value & 0xFF);
    buffer[1] = (uint8_t)((value >> 8) & 0xFF);
    buffer[2] = (uint8_t)((value >> 16) & 0xFF);
    buffer[3] = (uint8_t)((value >> 24) & 0xFF);
    buffer[4] = (uint8_t)((value >> 32) & 0xFF);
    buffer[5] = (uint8_t)((value >> 40) & 0xFF);
    buffer[6] = (uint8_t)((value >> 48) & 0xFF);
    buffer[7] = (uint8_t)((value >> 56) & 0xFF);

    int ret = dw1000_write(reg, buffer, sizeof(buffer));
    if (ret != 0)
    {
        LOG_ERR("Failed to write 64-bit value 0x%0llX to register 0x%0X", value, reg);
    }
    else
    {
        // LOG_INF("Wrote 64-bit value 0x%08X to register 0x%X", value, reg);
    }
    return ret;
}

// SPI write sub-address
int dw1000_subwrite(uint8_t reg, uint16_t subaddr, uint8_t *data, size_t len)
{
    uint8_t header[3];
    size_t header_len = 0;

    if (subaddr <= 0x7F)
    {
        // Short sub-index format: 2-byte header
        header[0] = 0xC0 | reg;     // Bit 7 = 1 (write), Bit 6 = 1 (subindex present)
        header[1] = subaddr & 0x7F; // Bit 7 = 0 (short format)
        header_len = 2;
    }
    else
    {
        // Long sub-index format: 3-byte header
        header[0] = 0xC0 | reg;              // Bit 7 = 1 (write), Bit 6 = 1 (subindex present)
        header[1] = 0x80 | (subaddr & 0x7F); // Bit 7 = 1 (long format), bits 6-0 = low 7 bits
        header[2] = (subaddr >> 7) & 0xFF;   // MSB of subaddress
        header_len = 3;
    }

    // Total TX length: header + data
    uint8_t tx_buf[7];
    memcpy(tx_buf, header, header_len);
    memcpy(tx_buf + header_len, data, len);

    struct spi_buf tx_bufs[] = {
        {.buf = tx_buf, .len = len + header_len},
    };
    struct spi_buf_set tx = {.buffers = tx_bufs, .count = 1};

    gpio_pin_set_dt(&cs_gpio, 0); // Assert CS
    int ret = spi_write_dt(&spispec, &tx);
    gpio_pin_set_dt(&cs_gpio, 1); // Deassert CS

    if (ret)
    {
        LOG_ERR("SPI sub-write failed: reg=0x%02X, sub=0x%04X, err=%d", reg, subaddr, ret);
    }

    return ret;
}

int dw1000_subwrite_u8(uint8_t reg, uint16_t subaddr, uint8_t value)
{
    return dw1000_subwrite(reg, subaddr, &value, 1);
    // LOG_INF("Wrote 8-bit value 0x%08X to register 0x%X:%0X", value, reg, subaddr);
}

int dw1000_subwrite_u16(uint8_t reg, uint16_t subaddr, uint16_t value)
{
    uint8_t buffer[2];
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    return dw1000_subwrite(reg, subaddr, buffer, sizeof(buffer));
    // LOG_INF("Wrote 16-bit value 0x%08X to register 0x%X:%0X", value, reg, subaddr);
}

int dw1000_subwrite_u32(uint8_t reg, uint16_t subaddr, uint32_t value)
{
    uint8_t buffer[4];
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
    return dw1000_subwrite(reg, subaddr, buffer, sizeof(buffer));
    // LOG_INF("Wrote 32-bit value 0x%08X to register 0x%X:%0X", value, reg, subaddr);
}

int dw1000_subwrite_u40(uint8_t reg, uint16_t subaddr, uint64_t value)
{
    uint8_t buffer[5];
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
    buffer[4] = (value >> 32) & 0xFF;
    return dw1000_subwrite(reg, subaddr, buffer, sizeof(buffer));
    // LOG_INF("Wrote 40-bit value 0x%08X to register 0x%X:%0X", value, reg, subaddr);
}

int dw1000_subwrite_u64(uint8_t reg, uint16_t subaddr, uint64_t value)
{
    uint8_t buffer[8];
    buffer[0] = value & 0xFF;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = (value >> 16) & 0xFF;
    buffer[3] = (value >> 24) & 0xFF;
    buffer[4] = (value >> 32) & 0xFF;
    buffer[5] = (value >> 40) & 0xFF;
    buffer[6] = (value >> 48) & 0xFF;
    buffer[7] = (value >> 56) & 0xFF;
    return dw1000_subwrite(reg, subaddr, buffer, sizeof(buffer));
    // LOG_INF("Wrote 64-bit value 0x%08X to register 0x%X:%0X", value, reg, subaddr);
}