#include "includes.h"

int idx = 0;

void check()
{
    uint32_t dev_id;
    dw1000_read_u32(0x00, &dev_id);
    LOG_INF("idx = %d %0X", idx, dev_id);
    idx++;
}

void bip_init()
{
    check(); // 0

    dw1000_write_u8(0x36, 0x01);
    check(); // 1

    dw1000_subwrite_u8(0x36, 0x01, 0x00);
    check(); //

    dw1000_subwrite_u16(0x36, 0x04, 0x00);

    check();

    // new_softreset();
    dw1000_subwrite_u16(AON_ID, 0x00, 0x00);
    dw1000_subwrite_u8(AON_ID, 0x06, 0x00);
    dw1000_subwrite_u8(AON_ID, 0x02, 0x00);
    dw1000_subwrite_u8(AON_ID, 0x02, 0x02); // initially 20
    dw1000_subwrite_u8(PMSC_ID, 3, 0x00);
    k_msleep(10);
    dw1000_subwrite_u8(PMSC_ID, 3, 0xF0);
    // - new_softreset();
    check();
    // new_enable_clocks(0);
    dw1000_write_u8(PMSC_ID, 0x01);
    dw1000_subwrite_u8(PMSC_ID, 0x1, 0x00);
    // - new_enable_clocks(0);
    check();
    dw1000_write_u8(EXT_SYNC_ID, 0x04);
    check();
    // uint32_t ldo_tune = new_otp_read(0x04);
    dw1000_subwrite_u16(OTP_IF_ID, 0x04, 0x0004);
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x03);
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x00);
    // - uint32_t ldo_tune = new_otp_read(0x04);
    check();
    // uint16_t otp_xtaltrim_and_rev = new_otp_read(0x1E) & 0xffff;
    dw1000_subwrite_u16(OTP_IF_ID, 0x04, 0x001E);
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x03);
    dw1000_subwrite_u8(OTP_IF_ID, OTP_CTRL, 0x00);
    // - uint16_t otp_xtaltrim_and_rev = new_otp_read(0x1E) & 0xffff;
    check();
    // new_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);
    dw1000_subwrite_u8(FS_CTRL_ID, 0x0E, 0x70);
    // - new_setxtaltrim((uint8_t)otp_xtaltrim_and_rev);
    check();
    dw1000_write_u8(0x36, 0x01);
    dw1000_subwrite_u8(0x36, 0x01, 0x03);
    check();
    // loks like enable clocks function - fast rate?
    dw1000_subwrite_u16(0x2d, 0x06, 0x8000);
    dw1000_write_u8(0x36, 0x00);
    check();
    dw1000_subwrite_u8(0x36, 0x01, 0x00);
    dw1000_write_u8(0x36, 0x00);
    dw1000_subwrite_u8(0x36, 0x01, 0x00);
    check();
    dw1000_subwrite_u8(AON_ID, 0x0A, 0x00);
}

void bip_config()
{

    dw1000_write_u32(SYS_CFG, 0x00000000);
    dw1000_subwrite_u16(LDE_IF_ID, 0x2804, 0x28F4);
    check();
    // new_configlde(prfIndex);
    dw1000_subwrite_u8(LDE_IF_ID, 0x0806, 0x6D);
    dw1000_subwrite_u16(LDE_IF_ID, 0x1806, (uint16_t)0x0607);
    check();
    // - new_configlde(prfIndex);

    dw1000_subwrite_u32(FS_CTRL_ID, 0x07, 0x0800041D);
    dw1000_subwrite_u8(FS_CTRL_ID, 0x0B, 0xBE);
    dw1000_subwrite_u8(RF_CONF_ID, 0x0B, 0xD8);
    check();
    dw1000_subwrite_u32(RF_CONF_ID, 0x0C, 0x001E3FE0);
    dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE0b, 0x0002);
    dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1a, 0x008D);
    dw1000_subwrite_u16(DRX_CONF_ID, DRX_TUNE1b, 0x0020);
    dw1000_subwrite_u8(DRX_CONF_ID, 0x26, 0x0028);
    check();
    dw1000_subwrite_u32(DRX_CONF_ID, DRX_TUNE2, 0x313B006B);
    dw1000_subwrite_u16(DRX_CONF_ID, 0x20, 0x00000081);
    dw1000_subwrite_u32(AGC_CFG_STS_ID, 0xC, 0x2502A907);
    dw1000_subwrite_u32(AGC_CFG_STS_ID, 0x4, 0x0000889B);
    dw1000_write_u8(USR_SFD_ID, 0x08);
    dw1000_write_u32(CHAN_CTRL, 0x4A7A0055);
    dw1000_write_u32(TX_FCTRL, 0x00164000);
    dw1000_write_u8(SYS_CTRL, 0x42); // Request TX start and TRX off at the same time
    check();
    dw1000_write_u32(0x26, 0x00001400);
    check();
    dw1000_subwrite_u32(0x36, 0x00, 0xF0300100); // repo - 0x00840000
    check();
    dw1000_subwrite_u32(0x36, 0x28, 0x00000110);
    check();
    dw1000_subwrite_u16(0x2E, 0x1804, 0x4034);
    check();
    dw1000_write_u16(0x18, 0x4034);
    check();
    // dw1000_write_u16(0x0D, 0x0100);
}
