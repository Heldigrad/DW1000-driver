#pragma once

#include "includes.h"

uint32_t partID;     // IC Part ID - read during initialisation
uint32_t lotID;      // IC Lot ID - read during initialisation
uint8_t vBatP;       // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
uint8_t tempP;       // IC V temp read during production and stored in OTP (Tmeas @ 23C)
uint8_t longFrames;  // Flag in non-standard long frame mode
uint8_t otprev;      // OTP revision number (read during initialisation)
uint32_t txFCTRL;    // Keep TX_FCTRL register config
uint32_t sysCFGreg;  // Local copy of system config register
uint8_t dblbuffon;   // Double RX buffer mode flag
uint8_t wait4resp;   // wait4response was set with last TX start command
uint16_t sleep_mode; // Used for automatic reloading of LDO tune and microcode at wake-up
uint16_t otp_mask;   // Local copy of the OTP mask used in dwt_initialise call

static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (129)            /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      * Used in RX only. */
};

uint16_t target[2] = {AGC_TUNE1_16M, AGC_TUNE1_64M};
const uint8_t dwnsSFDlen[] = {
    DW_NS_SFD_LEN_110K,
    DW_NS_SFD_LEN_850K,
    DW_NS_SFD_LEN_6M8};

void new_softreset()
{
    //_dwt_disablesequencing();

    // Clear any AON auto download bits (as reset will trigger AON download)
    dw1000_subwrite_u16(AON_ID, 0x00, 0x00);
    // Clear the wake-up configuration
    dw1000_subwrite_u8(AON_ID, 0x06, 0x00);
    // Upload the new configuration
    dw1000_subwrite_u8(AON_ID, 0x02, 0x00); // Clear the register
    dw1000_subwrite_u8(AON_ID, 0x02, 0x20);

    //     // Reset HIF, TX, RX and PMSC (set the reset bits)
    //     dw1000_subwrite_u8(PMSC_ID, 3, 0x00);

    //     // DW1000 needs a 10us sleep to let clk PLL lock after reset - the PLL will automatically lock after the reset
    //     // Could also have polled the PLL lock flag, but then the SPI needs to be < 3MHz !! So a simple delay is easier
    //     k_msleep(10);

    // Clear the reset bits
    dw1000_subwrite_u8(PMSC_ID, 3, 0xF0);
}

void new_enable_clocks(int clocks)
{
    uint8_t reg[2];

    dw1000_read(PMSC_ID, reg, sizeof(reg));

    switch (clocks)
    {
    case 1:
    {
        reg[0] = 0x00;
        reg[1] = reg[1] & 0xfe;
    }
    break;
    case 0:
    {
        // System and RX
        reg[0] = 0x01 | (reg[0] & 0xFC);
    }
    break;
    default:
        break;
    }

    // Need to write lower byte separately before setting the higher byte(s)
    dw1000_subwrite_u8(PMSC_ID, 0x00, reg[1]);
    dw1000_subwrite_u8(PMSC_ID, 0x1, reg[0]);
}

uint32_t new_otp_read(uint16_t address)
{
    uint32_t ret_data;

    // Write the address
    dw1000_subwrite_u16(OTP_IF_ID, 0x04, address);

    // Perform OTP Read - Manual read mode has to be set
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x0002 | 0x0001);
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x00); // OTPREAD is self clearing but OTPRDEN is not

    // Read read data, available 40ns after rising edge of OTP_READ
    dw1000_subread_u32(OTP_IF_ID, 0x0A, &ret_data);

    // Return the 32bit of read data
    return ret_data;
}

void new_setxtaltrim(uint8_t value)
{
    // The 3 MSb in this 8-bit register must be kept to 0b011 to avoid any malfunction.
    uint8_t reg_val = (3 << 5) | (value & 0x1F);
    dw1000_subwrite_u8(FS_CTRL_ID, 0x0E, reg_val);
}

void new_init()
{
    dblbuffon = 0;  // - set to 0 - meaning double buffer mode is off by default
    wait4resp = 0;  // - set to 0 - meaning wait for response not active
    sleep_mode = 0; // - set to 0 - meaning sleep mode has not been configured
    partID = 0;
    lotID = 0;
    vBatP = 0;
    tempP = 0;

    new_softreset();

    uint32_t dev_id;

    new_enable_clocks(0);

    // Configure the CPLL lock detect
    dw1000_subwrite_u8(EXT_SYNC_ID, 0x00, 0x04);

    // Load LDO tune from OTP and kick it if there is a value actually programmed.
    uint32_t ldo_tune = new_otp_read(0x04);
    if ((ldo_tune & 0xFF) != 0)
    {
        // Kick LDO tune
        dw1000_subwrite_u8(OTP_IF_ID, 0x12, 0x02); // Set load LDO kick bit
        sleep_mode |= 0x1000;                      // LDO tune must be kicked at wake-up
    }

    // Read OTP revision number
    uint16_t otp_xtaltrim_and_rev = new_otp_read(0x1E) & 0xffff; // Read 32 bit value, XTAL trim val is in low octet-0 (5 bits)
    otprev = (otp_xtaltrim_and_rev >> 8) & 0xff;                 // OTP revision is the next byte

    // XTAL trim
    if ((otp_xtaltrim_and_rev & 0x1F) == 0) // A value of 0 means that the crystal has not been trimmed
    {
        otp_xtaltrim_and_rev = 0x10; // Set to mid-range if no calibration value inside
    }
    // Configure XTAL trim
    new_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);

    // Load leading edge detect code (LDE/microcode)
    // Should disable the LDERUN bit enable if LDE has not been loaded
    // uint16_t rega;
    // dw1000_subread_u16(PMSC_ID, 0x04 + 1, &rega);
    // rega &= 0xFDFF; // Clear LDERUN bit
    // dw1000_subwrite_u16(PMSC_ID, 0x04 + 1, rega);

    // Enable clocks for sequencing
    new_enable_clocks(1);

    // The 3 bits in AON CFG1 register must be cleared to ensure proper
    // operation of the DW1000 in DEEPSLEEP mode.
    dw1000_subwrite_u8(AON_ID, 0x0A, 0x00);

    dw1000_read_u32(SYS_CFG, &sysCFGreg);                 // Read sysconfig register
    longFrames = (sysCFGreg & SYS_CFG_PHR_MODE_11) >> 16; // configure longFrames
    dw1000_read_u32(TX_FCTRL, &txFCTRL);

    // On Wake-up load the LDE microcode
    uint16_t AON_WCFG_reg;
    uint16_t ONW_LLDE_bit = (1 << 11);
    dw1000_subread_u16(0x2C, 0x00, &AON_WCFG_reg);
    AON_WCFG_reg |= ONW_LLDE_bit;
    dw1000_subwrite_u16(0x2C, 0x00, AON_WCFG_reg);

    // LDE_REPC
    dw1000_subwrite_u16(0x2E, 0x2804, 0x451E);
}

void new_configlde(int prfIndex)
{
    dw1000_subwrite_u8(LDE_IF_ID, 0x0806, (0x60) | (13)); // 8-bit configuration register

    if (prfIndex)
    {
        dw1000_subwrite_u16(LDE_IF_ID, 0x1806, (uint16_t)0x0607); // 16-bit LDE configuration tuning register
    }
    else
    {
        dw1000_subwrite_u16(LDE_IF_ID, 0x1806, (uint16_t)0x1607);
    }
}

void new_configure()
{
    uint8_t nsSfd_result = 0;
    uint8_t useDWnsSFD = 0;
    uint8_t chan = config.chan;
    uint32_t regval;
    uint16_t reg16 = 0x28F4; // PCODE 9
    uint8_t prfIndex = config.prf - DWT_PRF_16M;
    // uint8_t bw = ((chan == 4) || (chan == 7)) ? 1 : 0; // Select wide or narrow band

    // For 110 kbps we need a special setup
    if (DWT_BR_110K == config.dataRate)
    {
        sysCFGreg |= SYS_CFG_RXM110K;
        reg16 >>= 3; // lde_replicaCoeff must be divided by 8
    }
    else
    {
        sysCFGreg &= (~SYS_CFG_RXM110K);
    }

    longFrames = config.phrMode;

    sysCFGreg &= ~SYS_CFG_PHR_MODE_11;
    sysCFGreg |= (SYS_CFG_PHR_MODE_11 & ((uint32_t)config.phrMode << 16));

    dw1000_write_u32(SYS_CFG, sysCFGreg);

    // Set the lde_replicaCoeff
    dw1000_subwrite_u16(LDE_IF_ID, 0x2804, reg16);

    new_configlde(prfIndex);

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
    dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE0b, 0x0002);

    // DTUNE1
    dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1a, dtune1[prfIndex]);

    if (config.dataRate == DWT_BR_110K)
    {
        dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1b, 0x0064);
    }
    else
    {
        if (config.txPreambLength == DWT_PLEN_64)
        {
            dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1b, 0x0010);
            dw1000_subwrite_u8(DRX_CONF_ID, 0x26, 0x0010);
        }
        else
        {
            dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1b, 0x0020);
            dw1000_subwrite_u8(DRX_CONF_ID, 0x26, 0x0028);
        }
    }

    // DTUNE2
    dw1000_subwrite_u32(DRX_CONF_ID, DRX_TUNE2, digital_bb_config[prfIndex][config.rxPAC]);

    // DTUNE3 (SFD timeout)
    // Don't allow 0 - SFD timeout will always be enabled
    if (config.sfdTO == 0)
    {
        config.sfdTO = 0x1041;
    }
    dw1000_subwrite_u16(DRX_CONF_ID, 0x20, config.sfdTO);

    // Configure AGC parameters
    dw1000_subwrite_u32(AGC_CFG_STS_ID, 0xC, 0X2502A907UL);
    dw1000_subwrite_u32(AGC_CFG_STS_ID, 0x4, target[prfIndex]);

    // Set (non-standard) user SFD for improved performance,
    if (config.nsSFD)
    {
        // Write non standard (DW) SFD length
        dw1000_subwrite_u8(USR_SFD_ID, 0x00, dwnsSFDlen[config.dataRate]);
        nsSfd_result = 3;
        useDWnsSFD = 1;
    }
    regval = (CHAN_CTRL_TX_CHAN_MASK & (chan << CHAN_CTRL_TX_CHAN_SHIFT)) |                                 // Transmit Channel
             (CHAN_CTRL_RX_CHAN_MASK & (chan << CHAN_CTRL_RX_CHAN_SHIFT)) |                                 // Receive Channel
             (CHAN_CTRL_RXFPRF_MASK & ((uint32_t)config.prf << CHAN_CTRL_RXFPRF_SHIFT)) |                   // RX PRF
             ((CHAN_CTRL_TNSSFD | CHAN_CTRL_RNSSFD) & ((uint32_t)nsSfd_result << CHAN_CTRL_TNSSFD_SHIFT)) | // nsSFD enable RX&TX
             (CHAN_CTRL_DWSFD & ((uint32_t)useDWnsSFD << CHAN_CTRL_DWSFD_SHIFT)) |                          // Use DW nsSFD
             (CHAN_CTRL_TX_PCOD_MASK & ((uint32_t)config.txCode << CHAN_CTRL_TX_PCOD_SHIFT)) |              // TX Preamble Code
             (CHAN_CTRL_RX_PCOD_MASK & ((uint32_t)config.rxCode << CHAN_CTRL_RX_PCOD_SHIFT));

    dw1000_write_u32(CHAN_CTRL, regval);

    // Set up TX Preamble Size, PRF and Data Rate
    txFCTRL = ((uint32_t)(config.txPreambLength | config.prf) << 16) | ((uint32_t)config.dataRate << 13);
    dw1000_write_u32(TX_FCTRL, txFCTRL);

    // The SFD transmit pattern is initialised by the DW1000 upon a user TX request, but (due to an IC issue) it is not done for an auto-ACK TX. The
    // SYS_CTRL write below works around this issue, by simultaneously initiating and aborting a transmission, which correctly initialises the SFD
    // after its configuration or reconfiguration.
    // This issue is not documented at the time of writing this code. It should be in next release of DW1000 User Manual (v2.09, from July 2016).
    dw1000_subwrite_u8(SYS_CTRL, 0x00, SYS_CTRL_TXSTRT | SYS_CTRL_TRXOFF); // Request TX start and TRX off at the same time
}

void new_sync_rx_bufs()
{
    uint8_t buff;
    // Need to make sure that the host/IC buffer pointers are aligned before starting RX
    dw1000_subread_u8(SYS_STATUS, 3, &buff); // Read 1 byte at offset 3 to get the 4th byte out of 5

    if ((buff & (SYS_STATUS_ICRBP >> 24)) !=      // IC side Receive Buffer Pointer
        ((buff & (SYS_STATUS_HSRBP >> 24)) << 1)) // Host Side Receive Buffer Pointer
    {
        dw1000_subwrite_u8(SYS_CTRL_ID, SYS_CTRL_HRBT_OFFSET, 0x01); // We need to swap RX buffer status reg (write one to toggle internally)
    }
}

void new_rx_enable(int mode)
{
    uint16_t temp;
    uint8_t temp1;

    if ((mode & DWT_NO_SYNC_PTRS) == 0)
    {
        new_sync_rx_bufs();
    }

    temp = (uint16_t)SYS_CTRL_RXENAB;

    if (mode & DWT_START_RX_DELAYED)
    {
        temp |= (uint16_t)SYS_CTRL_RXDLYE;
    }

    dw1000_subwrite_u16(SYS_CTRL_ID, 0x00, temp);
}

void new_set_txfctrl(uint16_t txFrameLength, uint16_t txBufferOffset, int ranging)
{
    // Write the frame length to the TX frame control register
    // pdw1000local->txFCTRL has kept configured bit rate information
    uint32_t reg32 = txFCTRL | txFrameLength | ((uint32_t)txBufferOffset << TX_FCTRL_TXBOFFS_SHFT) | ((uint32_t)ranging << TX_FCTRL_TR_SHFT);
    dw1000_write_u32(TX_FCTRL, reg32);
}

void new_tx_start(int mode)
{
    uint8_t temp = 0x00;
    uint16_t checkTxOK = 0;

    if (mode & DWT_RESPONSE_EXPECTED)
    {
        temp = (uint8_t)SYS_CTRL_WAIT4RESP; // Set wait4response bit
        wait4resp = 1;
    }

    if (mode & DWT_START_TX_DELAYED)
    {
        // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
        temp |= (uint8_t)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
        dw1000_subwrite_u8(SYS_CTRL_ID, 0x00, temp);
    }
    else
    {
        temp |= (uint8_t)SYS_CTRL_TXSTRT;
        dw1000_subwrite_u8(SYS_CTRL_ID, 0x00, temp);
    }
}

void dw1000_soft_reset_gp()
{
    // Resetăm digitalul prin PMSC_CTRL0
    dw1000_subwrite_u16(PMSC, 0x00, 0x0000); // PMSC_CTRL0 = 0x0000 (clear)
    dw1000_subwrite_u16(PMSC, 0x00, 0x0001); // PMSC_CTRL0 = 0x0001 (set SYSCLKS = 1)

    // Resetăm blocurile digitale și analogice
    dw1000_subwrite_u8(PMSC, 0x03, 0x00); // PMSC_CTRL1_SUB = 0x00 (resetează și LDO, etc.)

    // Mică întârziere să se stabilizeze clock-ul
    k_busy_wait(10); // sau k_msleep(1);

    // Reset complet digital + analog (cum scrie în manual):
    dw1000_subwrite_u8(PMSC, 0x03, 0xF0); // PMSC_CTRL1 soft reset
    k_busy_wait(5);                       // Delay scurt
    dw1000_subwrite_u8(PMSC, 0x03, 0x00); // scoate resetul
}

void clear_regs()
{
    // dw1000_write_u32();
    // dw1000_subwrite_u32();
    dw1000_subwrite_u32(PMSC, PMSC_CTRL0, 0xF0300200);
    dw1000_subwrite_u32(PMSC, PMSC_CTRL1, 0x81020738);
    dw1000_subwrite_u16(0x2D, OTP_CTRL, 0x0000);
    dw1000_subwrite_u16(0x2D, OTP_ADDR, 0x0000);
    // TX_ANTD
    dw1000_subwrite_u8(0x2E, LDE_CFG1, 0x6C);
    // LDE_CFG2
    // DRX_TUNE2
    // DRX_TUNE4H
    // RF_RXCTRLH
    dw1000_subwrite_u32(0x28, RF_TXCTRL, 0x1E3DE0);
    // TC_PGDELAY
    // FS_PLLCFG
    // FS_PLLTUNE
    // FS_XTALT - nu l-as atinge
    dw1000_subwrite_u32(CHAN_CTRL, 0x00, 0x0055);
    dw1000_subwrite_u32(TX_FCTRL, 0x00, 0xC);
    dw1000_subwrite_u32(SYS_CFG, 0x00, 0x1200);
    // AGC_TUNE1
    dw1000_subwrite_u32(0x23, AGC_TUNE2, 0X2502A907);
    dw1000_subwrite_u16(0x23, AGC_TUNE3, 0x0035);
    // DRX_TUNE0b
    // DRX_TUNE1a
    // DRX_TUNE1b
    // DRX_SFDTOC
    // DRX_PRETOC
    // LDE_THRESH
    // TX_POWER
}