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

    // Clear status bit
    dw1000_write_u32(SYS_STATUS, SYS_STATUS_RX_OK | SYS_STATUS_ALL_RX_ERR);

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

void generic_default_configs(size_t frame_len)
{
    // CHAN_CTRL
    dw1000_write_u32(CHAN_CTRL, 0x21040055);

    // TX_FCTRL
    dw1000_write_u32(TX_FCTRL, 0x00154000 | (frame_len + 2)); // for a 4-byte message, tx_fctrl = 0x00154006

    // FS_CTRL
    dw1000_subwrite_u32(FS_CTRL, 0x07, 0x0800041D);
    dw1000_subwrite_u8(FS_CTRL, 0x0B, 0xBE);

    // GPT way
    // dw1000_write_u32(CHAN_CTRL, 0x25C86A5);

    // dw1000_subwrite_u8(0x0D, 0x00, 0x01); // TX standard SFD (SYS_CFG)

    // dw1000_write_u32(TX_FCTRL, 0x08200206); // Mode 2, 6.8 Mbps, preamble 128, 4-byte payload

    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE0b, 0x0016);    // PRF 16 MHz
    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1a, 0x008D);    // DRX_TUNE1a for 6.8 Mbps, PLEN 128
    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1b, 0x0020);    // Always 0x20 for 6.8 Mbps
    // dw1000_subwrite_u32(DRX_CONF, DRX_TUNE2, 0x0720A3DF); // ✅ correct

    // dw1000_subwrite_u16(0x2E, 0x1806, 0x0607); // LDE_CFG2 for PLEN 128

    // dw1000_write_u32(0x1E, 0x0E082848);   // TX_POWER
    // dw1000_subwrite_u8(0x2A, 0x0B, 0xC0); // TC_PGDELAY for ch5
    // dw1000_subwrite_u8(0x2B, 0x0B, 0xBE); // FS_PLLTUNE for ch5

    // // AGC tuning
    // dw1000_subwrite_u16(0x23, 0x04, 0x8870);
    // dw1000_subwrite_u32(0x23, 0x0C, 0x2502A907);

    // // RF_TXCTRL
    // dw1000_subwrite_u32(0x28, 0x0C, 0x001E3FE0);

    // // Load LDE microcode
    // load_lde_microcode(); // you already implemented this

    // uint16_t sfd_timeout = (128 + 1 + 8 - 8); // PLEN + SFD length - offset
    // dw1000_subwrite_u16(DRX_CONF, 0x20, sfd_timeout);
}

void tx_default_configs()
{
    // RF_TXCTRL
    dw1000_subwrite_u32(0x28, 0x0C, 0x001E3FE0);

    // Set standard preamble length, PRF 16 MHz, standard SFD, 110 kbps
    // (adjust if you want different settings)
    dw1000_subwrite_u8(TX_ANTD, 0x00, 0x10); // Typical TX antenna delay

    // Configure TX PHY parameters (same as RX must use)
    dw1000_subwrite_u32(TX_FCTRL, 0x00,
                        0x00000000 |       // PHR mode, non-extended frame
                            (0x01 << 20) | // PRF = 16 MHz (01)
                            (0x0A << 16) | // Preamble length = 128 symbols (0x0A)
                            (0x00 << 13) | // Data rate = 110 kbps
                            (4 + 2));      // Frame length + 2 CRC bytes
}

void rx_default_configs()
{
    // RF_RXCTRLH
    dw1000_subwrite_u8(RF_CONF, RF_RXCTRLH, 0xD8);

    // // DRX_TUNE0b
    // // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE0b, 0x0001); // my way i guess
    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE0b, 0x0016); // GPT way

    // // DRX_TUNE1a
    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1a, 0x0087);

    // // DRX_TUNE1b
    // dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1b, 0x0020);

    // // DRX_TUNE2
    // dw1000_subwrite_u32(DRX_CONF, DRX_TUNE2, 0x311A002D);

    // GPT way
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE0b, 0x0016);    // PRF 16 MHz
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1a, 0x008D);    // DRX_TUNE1a for 6.8 Mbps, PLEN 128
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1b, 0x0020);    // Always 0x20 for 6.8 Mbps
    dw1000_subwrite_u32(DRX_CONF, DRX_TUNE2, 0x0720A3DF); // For 6.8 Mbps and PRF 16
}

void rx_soft_reset(void)
{
    uint32_t ctrl0;
    dw1000_subread_u32(PMSC_CTRL0, PMSC_CTRL0_SOFTRESET, &ctrl0);

    ctrl0 &= ~SOFTRESET_RX_BIT;
    dw1000_subwrite_u32(PMSC_CTRL0, PMSC_CTRL0_SOFTRESET, ctrl0);

    k_busy_wait(5);

    ctrl0 |= SOFTRESET_RX_BIT;
    dw1000_subwrite_u32(PMSC_CTRL0, PMSC_CTRL0_SOFTRESET, ctrl0);
}

void load_lde_microcode()
{
    // 1. Enable LDE clock
    uint8_t lde_cfg = 0x01;
    dw1000_subwrite(0x36, 0x0B, &lde_cfg, 1); // PMSC_CTRL0_SUB:0B = 0x01

    // 2. Set LDELOAD bit (0x8000) in OTP_CTRL:06
    uint8_t otp_cmd[2] = {0x00, 0x80};       // Little-endian 0x8000
    dw1000_subwrite(0x2D, 0x06, otp_cmd, 2); // OTP_CTRL:06 = 0x8000

    // 3. Wait 150 µs
    k_busy_wait(150);

    // 4. Disable LDE clock
    lde_cfg = 0x00;
    dw1000_subwrite(0x36, 0x0B, &lde_cfg, 1); // PMSC_CTRL0_SUB:0B = 0x00
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
    dw1000_subwrite_u16(PMSC, PMSC_CTRL0, 0x0103);
    dw1000_subwrite_u16(OTP_IF, OTP_CTRL, 0x0080);
    k_busy_wait(150);
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
        // LOG_INF("Applying LDOTUNE calibration: 0x%02X", ldotune);
        dw1000_subwrite_u8(0x28, 0x30, ldotune); // REG: 0x28, SUB: 0x30
    }
    // else
    // {
    //     LOG_INF("No LDOTUNE calibration present in OTP");
    // }

    // LOG_INF("DW1000 default config applied.");
}

void print_enabled_bits(uint32_t value)
{
    int bits[32];
    int count = 0;

    for (int i = 0; i < 32; i++)
    {
        if (value & (1U << i))
        {
            bits[count++] = i;
        }
    }

    LOG_INF("Enabled bits:");

    for (int i = 0; i < 32; i++)
    {
        if (value & (1U << i))
        {
            const char *desc = bit_descriptions[i];
            if (desc && desc[0] != '\0')
            {
                printf("  Bit %2d: %s\n", i, desc);
            }
            else
            {
                printf("  Bit %2d: (no description)\n", i);
            }
        }
    }
}
