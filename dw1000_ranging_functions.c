#include "dw1000_ranging_functions.h"

void set_rx_antenna_delay(uint16_t rxDelay)
{
    dw1000_subwrite_u16(LDE_IF, LDE_RXANTD, rxDelay);
}

void set_tx_antenna_delay(uint16_t txDelay)
{
    dw1000_subwrite_u16(TX_ANTD, 0x00, txDelay);
}

uint64_t get_tx_timestamp()
{
    uint64_t tx_ts;
    dw1000_subread_u40(TX_TIME, 0x00, &tx_ts);

    return tx_ts;
}

uint64_t get_rx_timestamp()
{
    uint64_t rx_ts;
    dw1000_subread_u40(RX_TIME, 0x00, &rx_ts);

    return rx_ts;
}

int receive(uint64_t *buffer, uint64_t *timestamp)
{
    // LOG_INF("Starting receiver...");

    // Clear RX buffer
    dw1000_write_u64(RX_BUFFER, 0x00);
    uint32_t status_reg;
    new_rx_enable(0);

    do
    {
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR))
    {
        *timestamp = get_rx_timestamp();
        // LOG_INF("RX success! T2 = %08llX", *timestamp);
        dw1000_subread_u64(RX_BUFFER, 0x00, buffer);

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);

        return SUCCESS;
    }
    else
    {
        // LOG_INF("Errors encountered!");
        // print_enabled_bits(status_reg);

        /* Clear RX error events in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_ALL_RX_ERR);
        rx_soft_reset();
        return FAILURE;
    }
}

int transmit(uint64_t data, int len, uint64_t *timestamp)
{
    // LOG_INF("Starting transmitter...");

    dw1000_subwrite_u64(TX_BUFFER, 0x00, data);

    new_set_txfctrl(len);

    new_tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR));

    if (!(status & SYS_STATUS_ALL_TX_ERR))
    {
        *timestamp = get_tx_timestamp();
        // LOG_INF("TX success! T1 = %09llX", *timestamp);

        /* Clear TX frame sent event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK);

        return SUCCESS;
    }
    else
    {
        LOG_INF("Errors encountered!");
        print_enabled_bits(status);

        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return FAILURE;
    }
}

int32_t read_carrier_integrator(void)
{
    uint32_t regval = 0;
    int j;
    uint8_t buffer[DRX_CARRIER_INT_LEN];

    /* Read 3 bytes into buffer (21-bit quantity) */
    dw1000_subread(DRX_CONF, DRX_CARRIER_INT_OFFSET, buffer, DRX_CARRIER_INT_LEN);

    for (j = 2; j >= 0; j--) // arrange the three bytes into an unsigned integer value
    {
        regval = (regval << 8) + buffer[j];
    }

    if (regval & B20_SIGN_EXTEND_TEST)
        regval |= B20_SIGN_EXTEND_MASK; // sign extend bit #20 to whole word
    else
        regval &= DRX_CARRIER_INT_MASK; // make sure upper bits are clear if not sign extending

    return (int32_t)regval; // cast unsigned value to signed quantity.
}

double compute_distance_rep(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4)
{
    float clockOffsetRatio;
    uint64_t Tround, Treply, Tof;
    double distance;

    clockOffsetRatio = read_carrier_integrator();

    Tround = T4 - T1;
    Treply = T3 - T2;

    Tof = ((Tround - Treply * (1 - clockOffsetRatio)) / 2.0f) * (float)DWT_TIME_UNITS;

    distance = Tof * SPEED_OF_LIGHT;

    return distance;
}

double compute_distance(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4)
{
    uint64_t Tround, Treply, Tprop;
    if (T4 < T1)
    {
        // wrap-around correction (max 32-bit value is 0xFFFFFFFF)
        Tround = (uint64_t)(T4 + (1ULL << 32)) - T1;
    }
    else
    {
        Tround = T4 - T1;
    }
    if (T3 < T2)
    {
        // wrap-around correction (max 32-bit value is 0xFFFFFFFF)
        Tround = (uint64_t)(T3 + (1ULL << 32)) - T2;
    }
    else
    {
        // Time reply duration
        Treply = T3 - T2;
    }

    // One-way time of flight
    Tprop = (Tround - Treply) / 2;

    // Convert to seconds using DW1000 time units
    double tof_sec = Tprop * DWT_TIME_UNITS;

    // Compute distance
    double distance_m = tof_sec * SPEED_OF_LIGHT;

    return distance_m;
}

void set_rx_after_tx_delay(uint32_t rxDelayTime)
{
    uint32_t val;
    dw1000_read_u32(ACK_RESP_T, &val); // Read ACK_RESP_T_ID register

    val &= ~(ACK_RESP_T_W4R_TIM_MASK); // Clear the timer (19:0)

    val |= (rxDelayTime & ACK_RESP_T_W4R_TIM_MASK); // In UWB microseconds (e.g. turn the receiver on 20uus after TX)

    dw1000_write_u32(ACK_RESP_T, val);
}

void set_rx_timeout(uint16_t time)
{
    uint8_t temp;
    dw1000_subread_u8(SYS_CFG, 3, &temp);

    if (time > 0)
    {
        dw1000_subwrite_u8(RX_FWTO, 0x00, time);

        temp |= (uint8_t)(SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read the upper byte only

        dw1000_subwrite_u8(SYS_CFG, 3, temp); // Write at offset 3 to write the upper byte only
    }
    else
    {
        temp &= ~((uint8_t)(SYS_CFG_RXWTOE >> 24)); // Shift RXWTOE mask as we read the upper byte only

        dw1000_subwrite_u8(SYS_CFG, 3, temp); // Write at offset 3 to write the upper byte only
    }
}

void set_delayed_trx_time(uint32_t starttime)
{
    dw1000_subwrite_u32(DX_TIME, 1, starttime); // Write at offset 1 as the lower 9 bits of this register are ignored
}

uint64_t send_poll_message(uint8_t src_id, uint8_t dest_id)
{
    uint64_t tx_timestamp;
    uint32_t poll = ((uint32_t)POLL_MSG_TYPE << 24) |
                    ((uint32_t)src_id << 8) |
                    ((uint32_t)dest_id);

    transmit(poll, 4, &tx_timestamp);

    return tx_timestamp;
}

int get_poll_message(uint8_t src_id, uint8_t dest_id, uint64_t *timestamp)
{
    int res;
    uint8_t frame_len;
    uint32_t message = 0;
    res = receive(&message, timestamp);

    dw1000_subread_u8(RX_FINFO, 0x00, &frame_len); // Check frame length -> should be 12
    if (frame_len > 8)
    {
        return FAILURE;
    }

    return res;
    // LOG_INF("Received poll message = %0llX", message);

    uint8_t msg_type = (message >> 24) & 0xFF;
    uint8_t msg_src_id = (message >> 8) & 0xFF;
    uint8_t msg_dest_id = message & 0xFF;

    // return (msg_type == POLL_MSG_TYPE) &&
    //        (msg_dest_id == dest_id) &&
    //        (msg_src_id == src_id);

    return (msg_type == POLL_MSG_TYPE);
}

uint64_t send_resp_message(uint8_t src_id, uint8_t dest_id)
{
    uint64_t tx_timestamp;
    uint32_t resp = ((uint32_t)RESP_MSG_TYPE << 24) |
                    ((uint32_t)src_id << 8) |
                    ((uint32_t)dest_id);

    transmit(resp, 4, &tx_timestamp);
    return tx_timestamp;
}

int get_resp_message(uint8_t src_id, uint8_t my_id, uint64_t *timestamp)
{
    uint32_t message;
    return receive(&message, timestamp);

    // uint8_t msg_type = (message >> 24) & 0xFF;
    // uint8_t msg_src_id = (message >> 8) & 0xFF;
    // uint8_t msg_dest_id = message & 0xFF;

    // // return (msg_type == RESP_MSG_TYPE);
    // return SUCCESS;
}

int send_timestamps(uint8_t Dev_id, uint64_t T1, uint64_t T4)
{
    dw1000_subwrite_u8(TX_BUFFER, 0x07, FINAL_MSG_TYPE);
    dw1000_subwrite_u8(TX_BUFFER, 0x06, Dev_id);
    dw1000_subwrite_u40(TX_BUFFER, 0x05, T1);
    dw1000_subwrite_u40(TX_BUFFER, 0x00, T4);

    new_set_txfctrl(12); // 12 bytes: 2x5 bytes (timestamps) + dev_id + message_type

    new_tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR));

    if (!(status & SYS_STATUS_ALL_TX_ERR))
    {
        /* Clear TX frame sent event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK);

        return SUCCESS;
    }
    else
    {
        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return FAILURE;
    }
}

int get_timestamps(uint8_t Dev_id, uint64_t *T1, uint64_t *T4)
{
    uint32_t status_reg;
    uint8_t src_dev_id, msg_type, frame_len;
    new_rx_enable(0);

    do
    {
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG))
    {

        dw1000_subread_u8(RX_FINFO, 0x00, &frame_len); // Check frame length -> should be 12
        if (frame_len < 12)
        {
            return FAILURE;
        }

        dw1000_subread_u8(RX_BUFFER, 0x06, &src_dev_id);
        dw1000_subread_u40(RX_BUFFER, 0x05, T1);
        dw1000_subread_u40(RX_BUFFER, 0x00, T4);

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);

        return SUCCESS;
    }
    else
    {
        /* Clear RX error events in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_ALL_RX_ERR);
        rx_soft_reset();
        return FAILURE;
    }
}

int get_msg_from_init(uint64_t *T1, uint64_t *T2, uint64_t *T3, uint64_t *T4)
{
    uint32_t status_reg;
    uint8_t src_dev_id, frame_len;

    new_rx_enable(0);

    do
    {
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG))
    {
        // LOG_INF("Received message:");
        dw1000_subread_u8(RX_FINFO, 0x00, &frame_len); // Check frame length -> should be 12
        if (frame_len > 10)                            // TS message
        {
            dw1000_subread_u8(RX_BUFFER, 0x06, &src_dev_id);
            dw1000_subread_u40(RX_BUFFER, 0x05, T1);
            dw1000_subread_u40(RX_BUFFER, 0x00, T4);
            // LOG_INF("TS");
        }
        else
        {
            *T2 = get_rx_timestamp();
            k_msleep(10);
            // LOG_INF("POLL");
            *T3 = send_resp_message(0x01, 0x00);
        }

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);
    }
    else
    {
        /* Clear RX error events in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_ALL_RX_ERR);
        rx_soft_reset();
    }
}