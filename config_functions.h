#pragma once

#include "includes.h"

int dw1000_otp_read(uint16_t otp_addr, uint32_t *value)
{
    int ret;

    // Step 1: Write OTP address to OTP_ADDR (0x2D:04)
    ret = dw1000_subwrite_u16(0x2D, 0x04, otp_addr);
    if (ret)
        return ret;

    // Step 2: Issue OTPREAD command (set bit 1 of OTP_CTRL, 0x2D:06)
    ret = dw1000_subwrite_u16(0x2D, 0x06, 0x0002);
    if (ret)
        return ret;

    // Optional: wait briefly to ensure read completes
    k_busy_wait(5); // ~5µs delay

    // Step 3: Read result from OTP_RDAT (0x2D:0A)
    return dw1000_subread_u32(0x2D, 0x0A, value);
}

void rx_enable()
{
    uint16_t temp;

    uint8_t buff = 0;
    // Need to make sure that the host/IC buffer pointers are aligned before starting RX
    dw1000_subread_u8(SYS_STATUS, 3, &buff); // Read 1 byte at offset 3 to get the 4th byte out of 5

    if ((buff & (SYS_STATUS_ICRBP >> 24)) !=      // IC side Receive Buffer Pointer
        ((buff & (SYS_STATUS_HSRBP >> 24)) << 1)) // Host Side Receive Buffer Pointer
    {
        dw1000_subwrite_u8(SYS_CTRL, 3, 0x01); // Swap RX buffer status reg (write one to toggle internally)
    }

    temp = (uint16_t)SYS_CTRL_RXENAB;
    dw1000_subwrite_u16(SYS_CTRL, 0, temp);
}

void tx_start()
{
    uint8_t temp = 0x00;

    temp |= (uint8_t)SYS_CTRL_TXSTRT;
    dw1000_subwrite_u8(SYS_CTRL, 0, temp);
}

void generic_default_configs()
{
    // CHAN_CTRL
    dw1000_write_u32(CHAN_CTRL, 0x21040055);

    // TX_FCTRL
    dw1000_write_u32(TX_FCTRL, 0x00154006);

    // FS_CTRL
    dw1000_subwrite_u32(FS_CTRL, 0x07, 0x0800041D);
    dw1000_subwrite_u8(FS_CTRL, 0x0B, 0xBE);
}

void tx_default_configs()
{
    // RF_TXCTRL
    dw1000_subwrite_u32(0x28, 0x0C, 0x001E3FE0);
}

void rx_default_configs()
{
    // RF_RXCTRLH
    dw1000_subwrite_u8(0x28, 0x0B, 0xD8);

    // DRX_TUNE0b
    dw1000_subwrite_u16(0x27, 0x02, 0x0001);

    // DRX_TUNE1a
    dw1000_subwrite_u16(0x27, 0x04, 0x0087);

    // DRX_TUNE1b
    dw1000_subwrite_u16(0x27, 0x06, 0x0020);

    // DRX_TUNE2
    dw1000_subwrite_u32(0x27, 0x08, 0x311A002D);
}

void additional_default_configs()
{
    // Apply overrides
    dw1000_subwrite_u16(0x23, 0x04, 0x8870);     // AGC_TUNE1
    dw1000_subwrite_u32(0x23, 0x0C, 0x2502A907); // AGC_TUNE2
    dw1000_subwrite_u32(0x27, 0x08, 0x311A002D); // DRX_TUNE2
    dw1000_subwrite_u8(0x2E, 0x0806, 0x0D);      // NTM
    dw1000_subwrite_u16(0x2E, 0x1806, 0x1607);   // LDE_CFG2
    dw1000_write_u32(0x1E, 0x0E082848);          // TX_POWER
    dw1000_subwrite_u32(0x28, 0x0C, 0x001E3FE0); // RF_TXCTRL
    dw1000_subwrite_u8(0x2A, 0x0B, 0xC0);        // TC_PGDELAY
    dw1000_subwrite_u8(0x2B, 0x0B, 0xBE);        // FS_PLLTUNE

    // Load LDE microcode (LDELOAD)
    // dw1000_subwrite_u16(PMSC, PMSC_CTRL0, 0x0301);
    // dw1000_subwrite_u16(OTP_IF, OTP_CTRL, 0x8000);
    // k_busy_wait(1500);
    // dw1000_subwrite_u16(PMSC, PMSC_CTRL0, 0x0200);

    // Load LDE microcode (LDELOAD) - GPT way
    dw1000_subwrite_u16(PMSC, PMSC_CTRL0, 0x0100);
    dw1000_subwrite_u16(OTP_IF, OTP_CTRL, 0x8000);
    k_busy_wait(1500);
    dw1000_subwrite_u16(PMSC, PMSC_CTRL0, 0x0002);

    // uint8_t reg[2];
    // dw1000_subread(PMSC, 0x00, reg, sizeof(reg));
    // reg[0] = 0x01;
    // reg[1] = 0x03;
    // dw1000_subwrite_u8(PMSC_ID, 0x00, reg[0]);
    // dw1000_subwrite_u8(PMSC_ID, 0x01, reg[1]);

    // dw1000_subwrite_u16(OTP_IF_ID, OTP_CTRL, OTP_CTRL_LDELOAD); // Set load LDE kick bit

    // k_busy_wait(120); // Allow time for code to upload (should take up to 120 us)

    // uint8_t reg_1[2];
    // dw1000_subread(PMSC, 0x00, reg_1, sizeof(reg));
    // reg_1[0] = 0x00;
    // reg_1[1] = reg[1] & 0xfe;
    // dw1000_subwrite_u8(PMSC_ID, 0x00, reg[0]);
    // dw1000_subwrite_u8(PMSC_ID, 0x01, reg[1]);

    uint8_t address = 0x04;
    uint32_t otp_data;
    dw1000_otp_read(address, &otp_data);
    uint8_t ldotune = otp_data & 0xFF;
    if (ldotune != 0)
    {
        LOG_INF("Applying LDOTUNE calibration: 0x%02X", ldotune);
        dw1000_subwrite_u8(0x28, 0x30, ldotune); // REG: 0x28, SUB: 0x30
    }
    else
    {
        LOG_INF("No LDOTUNE calibration present in OTP");
    }

    LOG_INF("DW1000 default config applied.");
}
