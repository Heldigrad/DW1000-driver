#pragma once

#define SLEEP_TIME_MS 1000

// SPI Configuration
#define DW1000_SPI_FREQUENCY 2000000                    // 2 MHz
#define DW1000_SPI_MODE (SPI_MODE_CPOL | SPI_MODE_CPHA) // SPI Mode 0 (CPOL = 0, CPHA = 0)
#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB

// REGISTERS
#define DEV_ID 0x00
#define SYS_CFG 0x04
#define TX_BUFFER 0x09
#define SYS_CTRL 0x0D
#define SYS_STATUS 0x0F
#define RX_BUFFER 0x11
#define CHAN_CTRL 0x1F
#define PMSC 0x36
#define AON_ID 0x2C
#define OTP_MEM 0x2D
#define OTP_IF 0x2D
#define PMSC_CTRL0 0x00
#define OTP_CTRL 0x06
#define TX_FCTRL 0x08

//MISC
#define SYS_CTRL_TRXOFF 0x00000040
#define SYS_CTRL_SFTRST 0x00000001

#define SYS_STATUS_ICRBP 0x80000000UL /* IC side Receive Buffer Pointer READ ONLY */
#define SYS_STATUS_HSRBP 0x40000000UL /* Host Side Receive Buffer Pointer */
#define SYS_CTRL_RXENAB 0x00000100UL  /* Enable Receiver Now */
#define SYS_CTRL_TXSTRT 0x00000002UL /* Start Transmitting Now */

//#define SYS_STATUS_TXFRS 0x00000080UL /* Transmit Frame Sent: This is set when the transmitter has completed the sending of a frame */
#define SYS_STATUS_TXFRS (1 << 7)

//#define SYS_STATUS_RXFCG 0x00004000UL

#define SYS_STATUS_RXDFR (1 << 13)
#define SYS_STATUS_RXFCG (1 << 14)
/*
#define SYS_STATUS_RXFCG     (1 << 27) 
#define SYS_STATUS_CRCERR    (1 << 6)
#define SYS_STATUS_RXFCE     (1 << 7)
#define SYS_STATUS_RXFSL     (1 << 8)
#define SYS_STATUS_RXSTO     (1 << 9)
#define SYS_STATUS_RXPHE     (1 << 10)
#define SYS_STATUS_RXRFSL    (1 << 11)
#define SYS_STATUS_RXSFDTO   (1 << 12)
*/

#define SYS_STATUS_RXPHE (1 << 12) // Receiver PHY Header Error
#define SYS_STATUS_RXFCE (1 << 15) // Receiver FCS Error
#define SYS_STATUS_RXRFSL (1 << 16) // Receiver Reed Solomon Frame Sync Loss
#define SYS_STATUS_RXRFTO (1 << 17) // Receive Frame Wait Timeout
#define SYS_STATUS_LDEERR (1 << 18) // Leading edge detection processing error
#define SYS_STATUS_RXPTO (1 << 21) // Preamble detection timeout
#define SYS_STATUS_RXSFDTO (1 << 26) // Receive SFD timeout
#define SYS_STATUS_AFFREJ (1 << 29) // Automatic Frame Filtering rejection

#define SYS_STATUS_RX_OK (SYS_STATUS_RXDFR | SYS_STATUS_RXFCG)

#define SYS_STATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | \
    SYS_STATUS_RXRFSL | SYS_STATUS_RXRFTO | \
    SYS_STATUS_LDEERR | SYS_STATUS_RXPTO | \
    SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ)

/*
#define SYS_STATUS_RXPHE 0x00001000UL   // Receiver PHY Header Error 
#define SYS_STATUS_RXFCE 0x00008000UL   // Receiver FCS Error 
#define SYS_STATUS_RXRFSL 0x00010000UL  // Receiver Reed Solomon Frame Sync Loss 
#define SYS_STATUS_RXRFTO 0x00020000UL  // Receive Frame Wait Timeout 
#define SYS_STATUS_AFFREJ 0x20000000UL  // Automatic Frame Filtering rejection 
#define SYS_STATUS_RXSFDTO 0x04000000UL // Receive SFD timeout 
#define SYS_STATUS_LDEERR 0x00040000UL  // Leading edge detection processing error 

#define SYS_STATUS_ALL_RX_ERR (SYS_STATUS_RXPHE | SYS_STATUS_RXFCE | SYS_STATUS_RXRFSL | SYS_STATUS_RXSFDTO | SYS_STATUS_AFFREJ | SYS_STATUS_LDEERR)
*/

// FOR INIT AND CONFIG


#define EXT_SYNC_ID 0x24
#define OTP_IF_ID 0x2D
#define FS_CTRL_ID 0x2B
#define TX_FCTRL 0x08

#define XTRIM_ADDRESS 0x1E
#define LDOTUNE_ADDRESS 0x04
#define SYS_CFG_PHR_MODE_11 0x00030000UL

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

#define LDE_IF_ID 0x2E

#define RF_CONF_ID 0x28

#define DRX_CONF_ID 0x27

#define DRX_TUNE1a_PRF16 0x0087
#define DRX_TUNE1a_PRF64 0x008D

const uint16_t dtune1[2] =
    {
        DRX_TUNE1a_PRF16,
        DRX_TUNE1a_PRF64};

#define DRX_TUNE2_PRF16_PAC8 0x311A003CUL
#define DRX_TUNE2_PRF16_PAC16 0x331A0052UL
#define DRX_TUNE2_PRF16_PAC32 0x351A009AUL
#define DRX_TUNE2_PRF16_PAC64 0x371A011DUL
#define DRX_TUNE2_PRF64_PAC8 0x313B006BUL
#define DRX_TUNE2_PRF64_PAC16 0x333B00BEUL
#define DRX_TUNE2_PRF64_PAC32 0x353B015EUL
#define DRX_TUNE2_PRF64_PAC64 0x373B0296UL

const uint32_t digital_bb_config[2][4] =
    {
        {DRX_TUNE2_PRF16_PAC8,
         DRX_TUNE2_PRF16_PAC16,
         DRX_TUNE2_PRF16_PAC32,
         DRX_TUNE2_PRF16_PAC64},
        {DRX_TUNE2_PRF64_PAC8,
         DRX_TUNE2_PRF64_PAC16,
         DRX_TUNE2_PRF64_PAC32,
         DRX_TUNE2_PRF64_PAC64}};

#define AGC_CFG_STS_ID 0x23

#define AGC_TUNE1_16M 0x8870
#define AGC_TUNE1_64M 0x889B

uint16_t target[] = {AGC_TUNE1_16M, AGC_TUNE1_64M};

#define USR_SFD_ID 0x21

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

// for restoring to default
#define PMSC_ID 0x36
#define OTP_IF_ID 0x2D
#define PMSC_CTRL0 0x00
#define OTP_CTRL 0x06
#define SYS_CTRL_ID 0x0D
#define SYS_CTRL_TRXOFF 0x00000040
#define SYS_CTRL_SFTRST 0x00000001

typedef struct
{
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
    int cbData;          // Callback data structure
    int cbTxDone;        // Callback for TX confirmation event
    int cbRxOk;          // Callback for RX good frame event
    int cbRxTo;          // Callback for RX timeout events
    int cbRxErr;         // Callback for RX error events
} dwt_local_data_t;

// for configuration
typedef struct
{
    uint8_t chan;           //!< channel number {1, 2, 3, 4, 5, 7 }
    uint8_t prf;            //!< Pulse Repetition Frequency {DWT_PRF_16M or DWT_PRF_64M}
    uint8_t txPreambLength; //!< DWT_PLEN_64..DWT_PLEN_4096
    uint8_t rxPAC;          //!< Acquisition Chunk Size (Relates to RX preamble length)
    uint8_t txCode;         //!< TX preamble code
    uint8_t rxCode;         //!< RX preamble code
    uint8_t nsSFD;          //!< Boolean should we use non-standard SFD for better performance
    uint8_t dataRate;       //!< Data Rate {DWT_BR_110K, DWT_BR_850K or DWT_BR_6M8}
    uint8_t phrMode;        //!< PHR mode {0x0 - standard DWT_PHRMODE_STD, 0x3 - extended frames DWT_PHRMODE_EXT}
    uint16_t sfdTO;         //!< SFD timeout value (in symbols)
} dwt_config_t;

#define OTP_CTRL_LDELOAD        0x8000          /* This bit forces a load of LDE microcode */

// uint32_t tx_fctrl = 0;
//         tx_fctrl |= 4;           
//         tx_fctrl |= (0 << 10);  
//         tx_fctrl |= (0x2 << 11); 
//         tx_fctrl |= (0 << 13);   
//         tx_fctrl |= (0x5 << 18); 

//dw1000_write_u32(TX_FCTRL, tx_fctrl);