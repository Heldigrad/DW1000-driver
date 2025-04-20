#pragma once

#include "includes.h"

uint32_t otp_read(uint8_t address)
{

    // Write the address
    dw1000_subwrite_u8(OTP_IF_ID, 0x04, address);
    // Perform OTP Read - Manual read mode has to be set
    uint16_t otpread = 0x1100;
    dw1000_subwrite_u16(OTP_IF_ID, 0x06, otpread);
    uint8_t otpread_clear = 0x00;
    dw1000_subwrite_u8(OTP_IF_ID, 0x06, otpread_clear);
    // Read data, available 40ns after rising edge of OTP_READ
    uint32_t ldo_tune = 0;
    dw1000_subwrite_u32(OTP_IF_ID, 0x0A, ldo_tune);

    return ldo_tune;
}

dwt_local_data_t pdw1000local;

void initialize()
{

    pdw1000local.partID = 0;
    pdw1000local.lotID = 0;
    pdw1000local.vBatP = 0;
    pdw1000local.tempP = 0;
    pdw1000local.dblbuffon = 0;
    pdw1000local.wait4resp = 0;
    pdw1000local.sleep_mode = 0x1000;
    pdw1000local.otp_mask = 0;

    //_dwt_enableclocks(FORCE_SYS_XTI);
    uint8_t reg[2];
    uint32_t dev_id;
    dw1000_read_u32(0x00, &dev_id);
    dw1000_subread(PMSC, 0x00, reg, sizeof(reg));
    LOG_INF("b");
    reg[0] = 0x01 | (reg[0] & 0xfc);
    dw1000_subwrite_u8(PMSC, 0x00, reg[0]);
    dw1000_subwrite_u8(PMSC, 0x01, reg[1]);
    LOG_INF("b");
    //dwt_softreset();
    uint8_t softreset = 0x00;
    dw1000_subwrite_u8(PMSC, 0x04, softreset);

    dw1000_subwrite_u8(AON_ID, 0x00, softreset);
    dw1000_subwrite_u8(AON_ID, 0x06, softreset);
    dw1000_subwrite_u8(AON_ID, 0x02, softreset);
    uint8_t softreset_aux1 = 0x02;
    dw1000_subwrite_u8(AON_ID, 0x02, softreset_aux1);
    uint8_t softreset_aux2 = 0x00;
    dw1000_subwrite_u8(PMSC, 3, softreset_aux2);
    k_msleep(SLEEP_TIME_MS);
    uint8_t softreset_aux3 = 0xF0;
    dw1000_subwrite_u8(PMSC, 3, softreset_aux3);

    //Configure the CPLL lock detect
    uint8_t softreset_aux4 = 0x04;
    dw1000_subwrite_u8(EXT_SYNC_ID, 0x00, softreset_aux4);

    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    uint32_t ldo_tune = otp_read(LDOTUNE_ADDRESS);
    if ((ldo_tune & 0xFF) != 0)
    {
        // Kick LDO tune
        uint8_t kick_ldotune = 0x02;
        dw1000_subwrite_u8(OTP_IF_ID, 0x12, kick_ldotune);
        pdw1000local.sleep_mode |= 0x1000; // LDO tune must be kicked at wake-up
    }

    // Set OTP_CTRL LDELOAD bit
    dw1000_subwrite_u16(0x36, 0x00, 0x0301);
    dw1000_subwrite_u16(OTP_MEM, OTP_CTRL, 0x8000);
    k_msleep(5);
    dw1000_subwrite_u16(0x36, 0x00, 0x0200);

    // Read OTP revision number
    uint16_t otp_xtaltrim = otp_read(XTRIM_ADDRESS) & 0xffff; // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    pdw1000local.otprev = ((int)otp_xtaltrim >> 8) & 0xff;    // OTP revision is the next byte

    // XTAL trim value is set in OTP for DW1000 module and EVK/TREK boards but that might not be the case in a custom design
    if (((int)otp_xtaltrim & 0x1F) == 0) // A value of 0 means that the crystal has not been trimmed
    {
        otp_xtaltrim = 0x10; // Set to mid-range if no calibration value inside
    }

    // Configure XTAL trim
    uint8_t reg_val = (3 << 5) | (otp_xtaltrim & 0x1F);
    dw1000_subwrite_u8(FS_CTRL_ID, 0x0E, reg_val);

    uint8_t rega[2];
    dw1000_subread(PMSC, 0x04 + 1, rega, sizeof(rega));
    rega[0] &= 0xFF; // Clear LDERUN bit
    rega[1] &= 0xFD; // Clear LDERUN bit
    dw1000_subwrite(PMSC, 0x04 + 1, rega, sizeof(rega));

    // Enable clocks for sequencing
    uint8_t reg_2[2];
    reg_2[0] = 0x00;
    reg_2[1] = reg_2[1] & 0xFE;
    dw1000_subwrite_u8(PMSC, 0x00, reg_2[0]);
    dw1000_subwrite_u8(PMSC, 0x01, reg_2[1]);

    // The 3 bits in AON CFG1 register must be cleared to ensure proper operation of the DW1000 in DEEPSLEEP mode.
    uint8_t clear = 0x00;
    dw1000_subwrite_u8(AON_ID, 0x0A, clear);

    // Read system register / store local copy
    uint32_t ret;
    dw1000_read_u32(SYS_CFG, &ret);
    pdw1000local.sysCFGreg = ret;                                                   // Read sysconfig register
    pdw1000local.longFrames = (pdw1000local.sysCFGreg & SYS_CFG_PHR_MODE_11) >> 16; // configure longFrames
    dw1000_read_u32(TX_FCTRL, &ret);
    pdw1000local.txFCTRL = ret;

    LOG_INF("Init Done!");
}

dwt_config_t config = {
    5,               // Channel number.
DWT_PRF_64M,         // Pulse repetition frequency.
    DWT_PLEN_128,    // Preamble length. Used in TX only.
    DWT_PAC8,        // Preamble acquisition chunk size. Used in RX only.
    9,               // TX preamble code. Used in TX only.
    9,               // RX preamble code. Used in RX only.
    1,               // 0 to use standard SFD, 1 to use non-standard SFD.
    DWT_BR_6M8,      // Data rate.
    DWT_PHRMODE_EXT, // PHY header mode.
    (129)            // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};

void configure()
{
    uint8_t nsSfd_result = 0;
    uint8_t useDWnsSFD = 0;
    uint8_t chan = config.chan;
    uint32_t regval;
    uint16_t reg16 = 0x28F4;
    uint8_t prfIndex = config.prf - DWT_PRF_16M;
    //uint8_t bw = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band

    pdw1000local.sysCFGreg &= (~SYS_CFG_RXM110K);

    pdw1000local.longFrames = config.phrMode;

    pdw1000local.sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
    pdw1000local.sysCFGreg |= (SYS_CFG_PHR_MODE_11 & ((uint32_t)config.phrMode << 16));

    // Set OTP_CTRL LDELOAD bit
    dw1000_subwrite_u16(0x36, 0x00, 0x0301);
    dw1000_subwrite_u16(OTP_MEM, OTP_CTRL, 0x8000);
    k_msleep(5);
    dw1000_subwrite_u16(0x36, 0x00, 0x0200);

    dw1000_write_u32(SYS_CFG, pdw1000local.sysCFGreg);
    // Set the lde_replicaCoeff
    dw1000_subwrite_u16(LDE_IF_ID, 0x2804, reg16);
    dw1000_subwrite_u8(LDE_IF_ID, 0x0806, 0x60 | 13); // 8-bit configuration register

    if (prfIndex)
    {
        uint8_t lde_if[] = {0x07, 0x06};
        dw1000_subwrite(LDE_IF_ID, 0x1806, lde_if, sizeof(lde_if));
    }
    else
    {
        uint8_t lde_if[] = {0x07, 0x16};
        dw1000_subwrite(LDE_IF_ID, 0x1806, lde_if, sizeof(lde_if));
    }

    // Configure PLL2/RF PLL block CFG/TUNE (for a given channel)
    dw1000_subwrite_u32(FS_CTRL_ID, 0x07, 0x0800041DUL);
    dw1000_subwrite_u8(FS_CTRL_ID, 0x0B, 0xBE);

    // Configure RF RX blocks (for specified channel/bandwidth)
    dw1000_subwrite_u8(RF_CONF_ID, 0x0B, 0xD8);

    // Configure RF TX blocks (for specified channel and PRF)
    // Configure RF TX control
    dw1000_subwrite_u32(RF_CONF_ID, 0x0C, 0x001E3FE0UL);

    // Configure the baseband parameters (for specified PRF, bit rate, PAC, and SFD settings)
    // DTUNE0
    dw1000_subwrite_u16(DRX_CONF_ID, 0x02, 0x0001);

    // DTUNE1
    dw1000_subwrite_u16(DRX_CONF_ID, 0x04, dtune1[prfIndex]);

    dw1000_subwrite_u16(DRX_CONF_ID, 0x06, 0x0020);
    dw1000_subwrite_u16(DRX_CONF_ID, 0x26, 0x0028);

    // DTUNE2
    dw1000_subwrite_u32(DRX_CONF_ID, 0x08, digital_bb_config[prfIndex][config.rxPAC]);

    // DTUNE3 (SFD timeout)
    dw1000_subwrite_u32(DRX_CONF_ID, 0x20, config.sfdTO);

    // Configure AGC parameters
    dw1000_subwrite_u32(AGC_CFG_STS_ID, 0xC, 0X2502A907UL);
    dw1000_subwrite_u16(AGC_CFG_STS_ID, 0x4, target[prfIndex]);

    // Write non standard (DW) SFD length
    dw1000_subwrite_u8(USR_SFD_ID, 0x00, 8);

    nsSfd_result = 3;
    useDWnsSFD = 1;

    regval = (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) |                                 // Transmit Channel
             (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) |                                 // Receive Channel
             (CHAN_CTRL_RXFPRF_MASK & ((uint32_t)config.prf << CHAN_CTRL_RXFPRF_SHIFT)) |                   // RX PRF
             ((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD) & ((uint32_t)nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
             (CHAN_CTRL_DWSFD & ((uint32_t)useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) |                          // Use DW nsSFD
             (CHAN_CTRL_TX_PCOD_MASK & ((uint32_t)config.txCode << CHAN_CTRL_TX_PCOD_SHIFT)) |              // TX Preamble Code
             (CHAN_CTRL_RX_PCOD_MASK & ((uint32_t)config.rxCode << CHAN_CTRL_RX_PCOD_SHIFT));               // RX Preamble Code

    dw1000_write_u32(CHAN_CTRL, regval);

    // Set up TX Preamble Size, PRF and Data Rate
    pdw1000local.txFCTRL = ((uint32_t)(config.txPreambLength | config.prf) << 16) | ((uint32_t)config.dataRate << 13);
    dw1000_write_u32(TX_FCTRL, pdw1000local.txFCTRL);

    dw1000_subwrite_u32(SYS_CTRL, 0x00, 0x00000002UL | 0x00000040UL);
}