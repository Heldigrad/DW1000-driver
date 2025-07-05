#pragma once

// *******************************
//       SPI Configuration
// *******************************
#define DW1000_SPI_FREQUENCY 2000000                    // 2 MHz
#define DW1000_SPI_MODE (SPI_MODE_CPOL | SPI_MODE_CPHA) // SPI Mode 0 (CPOL = 0, CPHA = 0)
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB
// *******************************

// *******************************
//          REGISTERS
// *******************************
#define DEV_ID 0x00     // Device Identifier
#define SYS_CFG 0x04    // System Configuration bitmap
#define SYS_TIME 0x06   // System Time Counter (40-bit)
#define TX_FCTRL 0x08   // Transmit Frame Control
#define TX_BUFFER 0x09  // Transmit Data Buffer
#define DX_TIME 0x0A    // Delayed Send or Receive Time (40-bit)
#define RX_FWTO 0x0C    // Receive Frame Wait Timeout Period
#define SYS_CTRL 0x0D   // System Control Register
#define SYS_MASK 0x0E   // System Event Mask Register
#define SYS_STATUS 0x0F // System Event Status Register
#define RX_FINFO 0x10   // RX Frame Information
#define RX_BUFFER 0x11  // Receive Data Buffer
#define RX_TIME 0x15    // Receive Message Time of Arrival
#define TX_TIME 0x17    // Transmit Message Time of Sending
#define TX_ANTD 0x18    // 16-bit Delay from Transmit to Antenna
#define ACK_RESP_T 0x1A // Acknowledgement Time and Response Time
#define TX_POWER 0x1E   // TX Power Control
#define CHAN_CTRL 0x1F  // Channel Control
#define USR_SFD 0x21    // User-specified short/long TX/RX SFD sequences
#define AGC_CTRL 0x23   // Automatic Gain Control configuration
#define EXT_SYNC 0x24   // External synchronisation control
#define DRX_CONF 0x27   // Digital Receiver configuration
#define RF_CONF 0x28    // Analog RF Configuration
#define TC 0x2A         // Transmitter Calibration block
#define FS_CTRL 0x2B    // Frequency synthesiser control block
#define AON 0x2C        // Always-On register set
#define OTP_IF 0x2D     // One Time Programmable Memory Interface
#define LDE_IF 0x2E     // Leading Edge Detection Interface
#define PMSC 0x36       // Power Management System Control Block
// *******************************

// *******************************
//         SUBREGISTERS
// *******************************
// AON (0x2C) sub-registers
#define AON_CTRL 0x02 // AON Control Register

// OTP_IF (0x2D) sub-registers
#define OTP_CTRL 0x06 // OTP Control

// PMSC(0x36) sub-registers
#define PMSC_CTRL0 0x00 // PMSC Control Register 0

// FS_CTRL (0x2B) sub-registers
#define FS_PLLCFG 0x07  // Frequency synthesiser – PLL configuration
#define FS_PLLTUNE 0x0B // Frequency synthesiser – PLL Tuning
#define FS_XTALT 0x0E   // Frequency synthesiser – Crystal trim

// RF_CONF (0x28) sub-registers
#define RF_RXCTRLH 0x0B // Analog RX Control Register
#define RF_TXCTRL 0x0C  // Analog TX Control Register

// DRX_CONF (0x27) sub-registers
#define DRX_TUNE0b 0x02 // Digital Tuning Register 0b
#define DRX_TUNE1a 0x04 // Digital Tuning Register 1a
#define DRX_TUNE1b 0x06 // Digital Tuning Register 1b
#define DRX_TUNE2 0x08  // Digital Tuning Register 2
#define DRX_SFDTOC 0x20 // SFD timeout
#define DRX_PRETOC 0x24 // Preamble detection timeout
#define DRX_TUNE4H 0x26 // Digital Tuning Register 4H

// TC (0x2A) sub-registers
#define TC_PGDELAY 0x0B // Transmitter Calibration – Pulse Generator Delay

// AGC_CTRL (0x23) sub-registers
#define AGC_TUNE1 0x04 // AGC Tuning register 1
#define AGC_TUNE2 0x0C // AGC Tuning register 2
#define AGC_TUNE3 0x12 // AGC Tuning register 3

// LDE_IF (0x2E) sub-registers
#define LDE_THRESH 0x0000 // LDE Threshold report
#define LDE_RXANTD 0x1804 // LDE Receive Antenna Delay configuration
#define LDE_CFG2 0x1806   // LDE Configuration Register 2

// PMSC (0x36) sub-registers
#define PMSC_CTRL1 0x04 // PMSC Control Register 1
// *********************

// *******************************
//      SYS_STATUS BITS
// *******************************
#define SYS_STATUS_CPLOCK (1 << 1)     // Clock PLL Lock
#define SYS_STATUS_TXPUTE (1 << 2)     // TXPUTE - Transmit power up time error
#define SYS_STATUS_TXFRB (1 << 4)      // TXFRB - Transmit Frame Begins
#define SYS_STATUS_TXPRS (1 << 5)      // TXPRS - Transmit Preamble Sent
#define SYS_STATUS_TXPHS (1 << 6)      // TXPHS - Transmit PHY Header Sent
#define SYS_STATUS_TXFRS (1 << 7)      // TXFRS - Transmit Frame Sent
#define SYS_STATUS_LDEDONE (1 << 10)   // LDE processing done
#define SYS_STATUS_RXPHE (1 << 12)     // Receiver PHY Header Error
#define SYS_STATUS_RXDFR (1 << 13)     // Receiver Data Frame Ready
#define SYS_STATUS_RXFCG (1 << 14)     // Receiver FCS Good
#define SYS_STATUS_RXFCE (1 << 15)     // Receiver FCS Error
#define SYS_STATUS_RXRFSL (1 << 16)    // Receiver Reed Solomon Frame Sync Loss
#define SYS_STATUS_RXRFTO (1 << 17)    // Receive Frame Wait Timeout
#define SYS_STATUS_LDEERR (1 << 18)    // Leading edge detection processing error
#define SYS_STATUS_RXPTO (1 << 21)     // Preamble detection timeout
#define SYS_STATUS_SLP2INIT (1 << 23)  // SLEEP to INIT
#define SYS_STATUS_CLKPLL_LL (1 << 25) // Clock PLL Losing Lock
#define SYS_STATUS_RXSFDTO (1 << 26)   // Receive SFD timeout
#define SYS_STATUS_TXBERR (1 << 28)    // TXBERR - Transmit Buffer Error
#define SYS_STATUS_AFFREJ (1 << 29)    // Automatic Frame Filtering rejection

// SYS_STATUS RX good bits
#define SYS_STATUS_RX_OK (SYS_STATUS_RXDFR | SYS_STATUS_RXFCG | SYS_STATUS_LDEDONE)

// SYS_STATUS RX error bits
#define SYS_STATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE |    \
                               SYS_STATUS_RXRFSL | SYS_STATUS_RXRFTO |  \
                               SYS_STATUS_LDEERR | SYS_STATUS_RXPTO |   \
                               SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ | \
                               SYS_STATUS_CLKPLL_LL)

// SYS_STATUS TX good bits
#define SYS_STATUS_TX_OK (SYS_STATUS_TXFRB | SYS_STATUS_TXPRS | \
                          SYS_STATUS_TXPHS | SYS_STATUS_TXFRS)

// SYS_STATUS TX error bits
#define SYS_STATUS_ALL_TX_ERR (SYS_STATUS_TXPUTE | SYS_STATUS_TXBERR)

// *******************************

#define SUCCESS 0
#define FAILURE 1

// *******************************
//      CONFIG PARAMETERS
// *******************************

// SOFT RESET
#define PMSC_CTRL0_SOFTRESET 0x00
#define SOFTRESET_RX_BIT (1 << 28)

// SYS_CTRL FIELDS
#define SYS_CTRL_TRXOFF 0x00000040 // TRXOFF bit -> Transceiver Off (Any TX or RX activity will be aborted)

#define SYS_CTRL_RXENAB 0x00000100UL // Enable Receiver
#define SYS_CTRL_TXSTRT 0x00000002UL // Start Transmitting

#define DWT_TIME_UNITS (1.0 / 499.2e6 / 128.0)
#define SPEED_OF_LIGHT 299702547.0

// TODO: DETELEEEETETEE

#define DWT_START_RX_DELAYED 1       // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define SYS_CTRL_RXDLYE 0x00000200UL /* Receiver Delayed Enable (Enables Receiver when SY_TIME[0x??] == RXD_TIME[0x??] CHECK comment*/

#define DWT_RESPONSE_EXPECTED 2
#define SYS_CTRL_WAIT4RESP 0x00000080UL /* Wait for Response */
#define DWT_START_TX_DELAYED 1
#define SYS_CTRL_TXDLYS 0x00000004UL /* Transmitter Delayed Sending (initiates sending when SYS_TIME == TXD_TIME */

#define POLL1_MSG_TYPE 0xC5 // poll1 message - POLL
#define RESP1_MSG_TYPE 0xC6 // resp2 message - RESP
#define POLL2_MSG_TYPE 0xC7 // poll1 message - FINAL
#define RESP2_MSG_TYPE 0xC8 // resp2 message - REPORT

#define INF_LOGS_EN 0
#define ERR_LOGS_EN 0
#define DBG_LOGS_EN 0

#define LOG_INF_IF_ENABLED(...)   \
    do                            \
    {                             \
        if (INF_LOGS_EN)          \
            LOG_INF(__VA_ARGS__); \
    } while (0)

#define LOG_ERR_IF_ENABLED(...)   \
    do                            \
    {                             \
        if (ERR_LOGS_EN)          \
            LOG_ERR(__VA_ARGS__); \
    } while (0)

#define LOG_DBG_IF_ENABLED(...)   \
    do                            \
    {                             \
        if (DBG_LOGS_EN)          \
            LOG_DBG(__VA_ARGS__); \
    } while (0)

#define TAG_ID 0x00

#define ONE_SECOND_TICKS 998400000ULL            // ~ 1s in DW1000 time units
#define TWO_SECONDS_TICKS (ONE_SECOND_TICKS * 2) // ~ 2s in DW1000 time units

#define NR_OF_ANCHORS 4

#define ANCHOR_O 0x01
#define ANCHOR_X 0x02
#define ANCHOR_Y 0x03
#define ANCHOR_Z 0x04

#define DIST_X 3.0
#define DIST_Y 3.0
#define DIST_Z 2.25