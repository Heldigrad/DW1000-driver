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

#define TX_SLEEP_TIME_MS 500
#define RX_SLEEP_TIME_MS 500

// OTHER STUFF
#define PMSC_CTRL0_SOFTRESET 0x00
#define SOFTRESET_RX_BIT (1 << 28)

#define ACK_RESP_T_W4R_TIM_MASK 0x000FFFFFUL /* Wait-for-Response turn-around Time 20 bit field */

// MISC
#define SYS_CTRL_TRXOFF 0x00000040
#define SYS_CTRL_SFTRST 0x00000001

#define SYS_STATUS_ICRBP 0x80000000UL /* IC side Receive Buffer Pointer READ ONLY */
#define SYS_STATUS_HSRBP 0x40000000UL /* Host Side Receive Buffer Pointer */
#define SYS_CTRL_RXENAB 0x00000100UL  /* Enable Receiver Now */
#define SYS_CTRL_TXSTRT 0x00000002UL  /* Start Transmitting Now */

#define DWT_TIME_UNITS (1.0 / 499.2e6 / 128.0) //!< = 15.65e-12 s
#define SPEED_OF_LIGHT 299702547.0             // meters per second

#define SYS_CFG_RXWTOE 0x10000000UL /* Receive Wait Timeout Enable. */

//! constants for specifying the (Nominal) mean Pulse Repetition Frequency
//! These are defined for direct write (with a shift if necessary) to CHAN_CTRL and TX_FCTRL regs
#define DWT_PRF_16M 1 //!< UWB PRF 16 MHz
#define DWT_PRF_64M 2 //!< UWB PRF 64 MHz

//! constants for specifying Preamble Acquisition Chunk (PAC) Size in symbols
#define DWT_PAC8 0  //!< PAC  8 (recommended for RX of preamble length  128 and below
#define DWT_PAC16 1 //!< PAC 16 (recommended for RX of preamble length  256
#define DWT_PAC32 2 //!< PAC 32 (recommended for RX of preamble length  512
#define DWT_PAC64 3 //!< PAC 64 (recommended for RX of preamble length 1024 and up

//! constants for specifying TX Preamble length in symbols
//! These are defined to allow them be directly written into byte 2 of the TX_FCTRL register
//! (i.e. a four bit value destined for bits 20..18 but shifted left by 2 for byte alignment)
#define DWT_PLEN_4096 0x0C //! Standard preamble length 4096 symbols
#define DWT_PLEN_2048 0x28 //! Non-standard preamble length 2048 symbols
#define DWT_PLEN_1536 0x18 //! Non-standard preamble length 1536 symbols
#define DWT_PLEN_1024 0x08 //! Standard preamble length 1024 symbols
#define DWT_PLEN_512 0x34  //! Non-standard preamble length 512 symbols
#define DWT_PLEN_256 0x24  //! Non-standard preamble length 256 symbols
#define DWT_PLEN_128 0x14  //! Non-standard preamble length 128 symbols
#define DWT_PLEN_64 0x04   //! Standard preamble length 64 symbols

#define DWT_PHRMODE_STD 0x0 // standard PHR mode
#define DWT_PHRMODE_EXT 0x3 // DW proprietary extended frames PHR mode

#define DWT_BR_110K 0 //!< UWB bit rate 110 kbits/s
#define DWT_BR_850K 1 //!< UWB bit rate 850 kbits/s
#define DWT_BR_6M8 2  //!< UWB bit rate 6.8 Mbits/s

#define SYS_CFG_RXM110K 0x00400000 // UL
#define DRX_TUNE1a_PRF16 0x0087
#define DRX_TUNE1a_PRF64 0x008D

#define DRX_TUNE2_PRF16_PAC8 0x311A003CUL
#define DRX_TUNE2_PRF16_PAC16 0x331A0052UL
#define DRX_TUNE2_PRF16_PAC32 0x351A009AUL
#define DRX_TUNE2_PRF16_PAC64 0x371A011DUL
#define DRX_TUNE2_PRF64_PAC8 0x313B006BUL
#define DRX_TUNE2_PRF64_PAC16 0x333B00BEUL
#define DRX_TUNE2_PRF64_PAC32 0x353B015EUL
#define DRX_TUNE2_PRF64_PAC64 0x373B0296UL

#define AGC_CFG_STS_ID 0x23

#define AGC_TUNE1_16M 0x8870
#define AGC_TUNE1_64M 0x889B

/*mask and shift */
#define CHAN_CTRL_MASK 0xFFFF00FFUL         /* Channel Control Register access mask */
#define CHAN_CTRL_TX_CHAN_MASK 0x0000000FUL /* Supported channels are 1, 2, 3, 4, 5, and 7.*/
#define CHAN_CTRL_TX_CHAN_SHIFT (0)         /* Bits 0..3        TX channel number 0-15 selection */

#define CHAN_CTRL_RX_CHAN_MASK 0x000000F0UL
#define CHAN_CTRL_RX_CHAN_SHIFT (4) /* Bits 4..7        RX channel number 0-15 selection */

#define CHAN_CTRL_RXFPRF_MASK 0x000C0000UL /* Bits 18..19      Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_SHIFT (18)

/*offset 16 */
#define CHAN_CTRL_DWSFD 0x00020000UL /* Bit 17 This bit enables a non-standard DecaWave proprietary SFD sequence. */
#define CHAN_CTRL_DWSFD_SHIFT (17)
#define CHAN_CTRL_TNSSFD 0x00100000UL /* Bit 20 Non-standard SFD in the transmitter */
#define CHAN_CTRL_TNSSFD_SHIFT (20)
#define CHAN_CTRL_RNSSFD 0x00200000UL /* Bit 21 Non-standard SFD in the receiver */
#define CHAN_CTRL_RNSSFD_SHIFT (21)

/* Specific RXFPRF configuration */
#define CHAN_CTRL_RXFPRF_4 0x00000000UL     /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_16 0x00040000UL    /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_RXFPRF_64 0x00080000UL    /* Specify (Force) RX Pulse Repetition Rate: 00 = 4 MHz, 01 = 16 MHz, 10 = 64MHz. */
#define CHAN_CTRL_TX_PCOD_MASK 0x07C00000UL /* Bits 22..26      TX Preamble Code selection, 1 to 24. */
#define CHAN_CTRL_TX_PCOD_SHIFT (22)
#define CHAN_CTRL_RX_PCOD_MASK 0xF8000000UL /* Bits 27..31      RX Preamble Code selection, 1 to 24. */
#define CHAN_CTRL_RX_PCOD_SHIFT (27)

#define SYS_STATUS_ICRBP 0x80000000UL /* IC side Receive Buffer Pointer READ ONLY */
#define SYS_STATUS_HSRBP 0x40000000UL /* Host Side Receive Buffer Pointer */
#define SYS_CTRL_RXENAB 0x00000100UL  /* Enable Receiver Now */

#define SYS_CTRL_TXSTRT 0x00000002UL /* Start Transmitting Now */

#define OTP_CTRL_LDELOAD 0x8000 /* This bit forces a load of LDE microcode */

#define DW_NS_SFD_LEN_110K 64 /* Decawave non-standard SFD length for 110 kbps */
#define DW_NS_SFD_LEN_850K 16 /* Decawave non-standard SFD length for 850 kbps */
#define DW_NS_SFD_LEN_6M8 8   /* Decawave non-standard SFD length for 6.8 Mbps */

#define SYS_CTRL_HRBT_OFFSET (3)
#define DWT_START_RX_DELAYED 1          // Set up delayed RX, if "late" error triggers, then the RX will be enabled immediately
#define SYS_CTRL_RXDLYE 0x00000200UL    /* Receiver Delayed Enable (Enables Receiver when SY_TIME[0x??] == RXD_TIME[0x??] CHECK comment*/
#define SYS_STATUS_HPDWARN 0x08000000UL /* Half Period Delay Warning */
#define DWT_IDLE_ON_DLY_ERR 2           // If delayed RX failed due to "late" error then if this
#define TX_FCTRL_TXBOFFS_SHFT (22)
#define TX_FCTRL_TR_SHFT (15)
#define DWT_RESPONSE_EXPECTED 2
#define SYS_CTRL_WAIT4RESP 0x00000080UL /* Wait for Response */
#define DWT_START_TX_DELAYED 1
#define SYS_CTRL_TXDLYS 0x00000004UL /* Transmitter Delayed Sending (initiates sending when SYS_TIME == TXD_TIME */
#define SYS_STATUS_TXERR (0x0408)    /* These bits are the 16 high bits of status register TXPUTE and HPDWARN flags */

#define TX_ANT_DLY 15222
#define RX_ANT_DLY 15222

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 140
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 1000

/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 630
#define UUS_TO_DWT_TIME 65536

#define DRX_CARRIER_INT_OFFSET 0x28
#define DRX_CARRIER_INT_LEN (3)
#define DRX_CARRIER_INT_MASK 0x001FFFFF

#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)

#define POLL_MSG_TYPE 0xC5  // Custom type for poll message
#define RESP_MSG_TYPE 0xC6  // Distinct from POLL_MSG_TYPE
#define FINAL_MSG_TYPE 0xC7 // Unique type for final message

#define POLL_MSG_LEN 4 // 1 byte type + 2 + 2 + 2

#define NR_OF_DISTANCES 10