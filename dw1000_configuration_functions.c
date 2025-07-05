#include "dw1000_configuration_functions.h"

// SYS_STATUS BITs DESCRIPTION
const char *bit_descriptions[32] = {
    "IRQS   - Interrupt Request Status",
    "CPLOCK - Clock PLL Lock",
    "ESYNCR - External Sync Clock Reset",
    "AAT    - Automatic Acknowledge Trigger",
    "TXFRB  - Transmit Frame Begins",
    "TXPRS  - Transmit Preamble Sent",
    "TXPHS  - Transmit PHY Header Sent",
    "TXFRS  - Transmit Frame Sent",
    "RXPRD  - Receiver Preamble Detected status",
    "RXSFDD - Receiver SFD Detected",
    "LDEDONE - LDE processing done",
    "RXPHD  - Receiver PHY Header Detect",
    "RXPHE  - Receiver PHY Header Error",
    "RXDFR  - Receiver Data Frame Ready",
    "RXFCG  - Receiver FCS Good",
    "RXFCE  - Receiver FCS Error",
    "RXRFSL - Receiver Reed Solomon Frame Sync Loss",
    "RXRFTO - Receive Frame Wait Timeout",
    "LDEERR - Leading edge detection processing error",
    "R      - Reserved",
    "RXOVRR - Receiver Overrun",
    "RXPTO  - Preamble detection timeout",
    "GPIOIRQ - GPIO interrupt",
    "SLP2INIT - SLEEP to INIT",
    "RFPLL_LL - RF PLL Losing Lock",
    "CLKPLL_LL - Clock PLL Losing Lock",
    "RXSFDTO - Receive SFD timeout",
    "HPDWARN - Half Period Delay Warning",
    "TXBERR  - Transmit Buffer Error",
    "AFFREJ  - Automatic Frame Filtering rejection",
    "HSRBP   - Host Side Receive Buffer Pointer",
    "ICRBP   - IC side Receive Buffer Pointer"};

void initialize()
{
    dw1000_write_u8(0x36, 0x01);
    dw1000_subwrite_u8(0x36, 0x01, 0x00);
    dw1000_subwrite_u16(0x36, 0x04, 0x00);

    // new_softreset();
    dw1000_subwrite_u16(AON, 0x00, 0x00);
    dw1000_subwrite_u8(AON, 0x06, 0x00);
    dw1000_subwrite_u8(AON, 0x02, 0x00);
    dw1000_subwrite_u8(AON, 0x02, 0x02);
    dw1000_subwrite_u8(PMSC, 3, 0x00);
    k_msleep(10);
    dw1000_subwrite_u8(PMSC, 3, 0xF0);
    // - new_softreset();

    // new_enable_clocks(0);
    dw1000_write_u8(PMSC, 0x01);
    dw1000_subwrite_u8(PMSC, 0x1, 0x00);
    // - new_enable_clocks(0);

    dw1000_write_u8(EXT_SYNC, 0x04);

    // uint32_t ldo_tune = new_otp_read(0x04);
    dw1000_subwrite_u16(OTP_IF, 0x04, 0x0004);
    dw1000_subwrite_u8(OTP_IF, OTP_CTRL, 0x03);
    dw1000_subwrite_u8(OTP_IF, OTP_CTRL, 0x00);
    // - uint32_t ldo_tune = new_otp_read(0x04);

    // uint16_t otp_xtaltrim_and_rev = new_otp_read(0x1E) & 0xffff;
    dw1000_subwrite_u16(OTP_IF, 0x04, 0x001E);
    dw1000_subwrite_u8(OTP_IF, OTP_CTRL, 0x03);
    dw1000_subwrite_u8(OTP_IF, OTP_CTRL, 0x00);
    // - uint16_t otp_xtaltrim_and_rev = new_otp_read(0x1E) & 0xffff;

    // new_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);
    dw1000_subwrite_u8(FS_CTRL, 0x0E, 0x70);
    // - new_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);

    dw1000_write_u8(0x36, 0x01);
    dw1000_subwrite_u8(0x36, 0x01, 0x03);

    dw1000_subwrite_u16(0x2d, 0x06, 0x8000);
    dw1000_write_u8(0x36, 0x00);
    dw1000_subwrite_u8(0x36, 0x01, 0x00);
    dw1000_write_u8(0x36, 0x00);
    dw1000_subwrite_u8(0x36, 0x01, 0x00);

    dw1000_subwrite_u8(AON, 0x0A, 0x00);
}

void configure()
{

    dw1000_write_u32(SYS_CFG, 0x00000000 | (1 << 12));
    dw1000_subwrite_u16(LDE_IF, 0x2804, 0x28F4);

    // new_configlde(prfIndex);
    dw1000_subwrite_u8(LDE_IF, 0x0806, 0x6D);
    dw1000_subwrite_u16(LDE_IF, 0x1806, (uint16_t)0x0607);
    // - new_configlde(prfIndex);

    dw1000_subwrite_u32(FS_CTRL, 0x07, 0x0800041D);
    dw1000_subwrite_u8(FS_CTRL, 0x0B, 0xBE);
    dw1000_subwrite_u8(RF_CONF, 0x0B, 0xD8);

    dw1000_subwrite_u32(RF_CONF, 0x0C, 0x001E3FE0);
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE0b, 0x0002);
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1a, 0x008D);
    dw1000_subwrite_u16(DRX_CONF, DRX_TUNE1b, 0x0020);
    dw1000_subwrite_u8(DRX_CONF, 0x26, 0x0028);

    dw1000_subwrite_u32(DRX_CONF, DRX_TUNE2, 0x313B006B);
    dw1000_subwrite_u16(DRX_CONF, 0x20, 0x00000081);
    dw1000_subwrite_u32(AGC_CTRL, 0xC, 0x2502A907);
    dw1000_subwrite_u32(AGC_CTRL, 0x4, 0x0000889B);
    dw1000_write_u8(USR_SFD, 0x08);
    dw1000_write_u32(CHAN_CTRL, 0x4A7A0055);
    dw1000_write_u32(TX_FCTRL, 0x00164000);
    // dw1000_write_u8(SYS_CTRL, 0x42); // Request TX start and TRX off at the same time

    dw1000_write_u32(0x26, 0x00001400);

    dw1000_subwrite_u32(0x36, 0x00, 0xF0300100); // repo - 0x00840000

    dw1000_subwrite_u32(0x36, 0x28, 0x00000110);

    dw1000_subwrite_u16(0x2E, 0x1804, 0x4034);

    dw1000_write_u16(0x18, 0x4034);
}

void set_txfctrl(uint16_t txFrameLength)
{
    uint32_t val = 0x16C000 | (txFrameLength + 2);
    dw1000_write_u32(TX_FCTRL, val);
}

void rx_enable(int mode)
{
    uint16_t temp;

    temp = (uint16_t)SYS_CTRL_RXENAB;

    if (mode & DWT_START_RX_DELAYED)
    {
        temp |= (uint16_t)SYS_CTRL_RXDLYE;
    }

    dw1000_subwrite_u16(SYS_CTRL, 0x00, temp);
}

void tx_start(int mode)
{
    uint8_t temp = 0x00;

    if (mode & DWT_RESPONSE_EXPECTED) // 2
    {
        temp = (uint8_t)SYS_CTRL_WAIT4RESP; // Set wait4response bit
    }

    if (mode & DWT_START_TX_DELAYED) // 1
    {
        // Both SYS_CTRL_TXSTRT and SYS_CTRL_TXDLYS to correctly enable TX
        temp |= (uint8_t)(SYS_CTRL_TXDLYS | SYS_CTRL_TXSTRT);
    }
    else
    {
        temp |= (uint8_t)SYS_CTRL_TXSTRT;
    }

    dw1000_subwrite_u8(SYS_CTRL, 0x00, temp);
}

void rx_soft_reset(void)
{
    uint32_t ctrl0;
    dw1000_subread_u32(PMSC, PMSC_CTRL0_SOFTRESET, &ctrl0);

    ctrl0 &= (uint32_t)(~SOFTRESET_RX_BIT);
    dw1000_subwrite_u32(PMSC, PMSC_CTRL0_SOFTRESET, ctrl0);

    k_busy_wait(5);

    ctrl0 = ctrl0 | (uint32_t)SOFTRESET_RX_BIT;

    dw1000_subwrite_u32(PMSC, PMSC_CTRL0_SOFTRESET, ctrl0);
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

void test_spi()
{
    uint32_t dev_id;
    dw1000_read_u32(DEV_ID, &dev_id);
    LOG_INF("dev_id = %0X", dev_id);
}
