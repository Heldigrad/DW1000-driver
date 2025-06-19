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
    dw1000_subwrite_u64(TX_BUFFER, 0x00, data);

    new_set_txfctrl(len);

    new_tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & (SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR)));

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
        regval &= DRX_CARRIER_INT_MASK; // make sure upper bits are clear if not
                                        // sign extending

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
        Treply = (uint64_t)(T3 + (1ULL << 32)) - T2;
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

        temp |= (uint8_t)(SYS_CFG_RXWTOE >> 24); // Shift RXWTOE mask as we read
                                                 // the upper byte only

        dw1000_subwrite_u8(SYS_CFG, 3, temp); // Write at offset 3 to write the upper byte only
    }
    else
    {
        temp &= ~((uint8_t)(SYS_CFG_RXWTOE >> 24)); // Shift RXWTOE mask as we read the upper byte only
        dw1000_subwrite_u8(SYS_CFG, 3, temp);       // Write at offset 3 to write the upper byte only
    }
}

void set_delayed_trx_time(uint32_t starttime)
{
    dw1000_subwrite_u32(DX_TIME, 1, starttime); // Write at offset 1 as the lower 9 bits of this register are ignored
}

uint64_t send_poll_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id)
{
    uint64_t tx_timestamp;
    uint32_t poll = ((uint32_t)POLL_MSG_TYPE << 24) |
                    ((uint32_t)src_id << 16) |
                    ((uint32_t)dest_id << 8) |
                    (uint32_t)message_id;

    transmit(poll, 4, &tx_timestamp);

    return tx_timestamp;
}

int get_poll_message(uint8_t src_id, uint8_t dest_id, uint64_t *timestamp)
{
    int res;
    uint8_t frame_len;
    uint64_t message = 0;
    res = receive(&message, timestamp);

    dw1000_subread_u8(RX_FINFO, 0x00,
                      &frame_len); // Check frame length -> should be 12
    if (frame_len > 8)
    {
        return FAILURE;
    }

    return res;
    // LOG_INF("Received poll message = %0llX", message);

    uint8_t msg_type = (message >> 24) & 0xFF;
    // uint8_t msg_src_id = (message >> 8) & 0xFF;
    // uint8_t msg_dest_id = message & 0xFF;

    // return (msg_type == POLL_MSG_TYPE) &&
    //        (msg_dest_id == dest_id) &&
    //        (msg_src_id == src_id);

    return (msg_type == POLL_MSG_TYPE);
}

uint64_t send_resp_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id)
{
    uint64_t tx_timestamp;
    uint32_t resp = ((uint32_t)RESP_MSG_TYPE << 24) |
                    ((uint32_t)src_id << 16) |
                    ((uint32_t)dest_id << 8) |
                    (uint32_t)message_id;

    transmit(resp, 4, &tx_timestamp);
    return tx_timestamp;
}

uint8_t get_rx_frame_len()
{
    uint8_t frame_len;
    dw1000_subread_u8(RX_FINFO, 0x00, &frame_len);
    return frame_len & 0x7F;
}

int get_resp_message(uint8_t my_id, uint8_t src_id, uint8_t message_id, uint64_t *timestamp)
{
    uint64_t message;
    int ret = receive(&message, timestamp);

    if (ret == SUCCESS)
    {
        uint8_t msg_type = (message >> 24) & 0xFF;

        if (msg_type == RESP_MSG_TYPE)
        {
            uint8_t rec_src_id = (message >> 16) & 0xFF;
            uint8_t dest_id = (message >> 8) & 0xFF;

            if (rec_src_id == src_id && dest_id == my_id)
            {
                uint8_t msg_id = (message) & 0xFF;
                if (msg_id != message_id)
                {
                    if (INFO_LOGS_EN)
                    {
                        LOG_INF("Expected resp: %0d, received: %0d", message_id, msg_id);
                    }
                    return FAILURE;
                }
            }
        }
        else
        {
            uint8_t src_id = (message >> 16) & 0xFF;
            if (src_id == src_id)
            {
                uint32_t distance_mm = message & 0xFFFFFFFF;

                double distance = (double)distance_mm / 1000.0; // convert to m

                LOG_INF("Distance = %0.2fm", distance);
            }

            return FAILURE; // so that the timestamp isn't used in ranging
        }
    }

    return ret;
}

int send_timestamps(uint8_t Src_id, uint8_t Dest_id, uint64_t T1, uint64_t T4, uint8_t message_id)
{
    dw1000_subwrite_u8(TX_BUFFER, 0x0D, TS_MSG_TYPE);
    dw1000_subwrite_u8(TX_BUFFER, 0x0C, Src_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x0B, Dest_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x0A, message_id);
    dw1000_subwrite_u40(TX_BUFFER, 0x05, T1);
    dw1000_subwrite_u40(TX_BUFFER, 0x00, T4);

    new_set_txfctrl(14); // 14 bytes: 2x5 bytes (timestamps) + dev_id + dest_id + message_type + message_id

    new_tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & (SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR)));

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
    uint8_t src_dev_id, frame_len;
    new_rx_enable(0);

    do
    {
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG))
    {
        dw1000_subread_u8(RX_FINFO, 0x00,
                          &frame_len); // Check frame length -> should be 12
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

void get_msg_from_init(uint8_t my_id, uint64_t *T1, uint64_t *T2, uint64_t *T3, uint64_t *T4, uint8_t *message_id)
{
    uint32_t status_reg, buffer;
    uint8_t msg_id, src_id, dest_id, frame_len;

    new_rx_enable(0);

    do
    {
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while (!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG))
    {
        dw1000_subread_u8(RX_FINFO, 0x00, &frame_len); // Check frame length -> should be 14

        if (frame_len > 10) // TS message
        {
            dw1000_subread_u8(RX_BUFFER, 0x0C, &src_id);
            dw1000_subread_u8(RX_BUFFER, 0x0B, &dest_id);
            dw1000_subread_u8(RX_BUFFER, 0x0A, &msg_id);

            if (dest_id == my_id)
            {

                if (*message_id == msg_id)
                {
                    dw1000_subread_u40(RX_BUFFER, 0x05, T1);
                    dw1000_subread_u40(RX_BUFFER, 0x00, T4);

                    if (INFO_LOGS_EN)
                    {
                        LOG_INF("Timestamps received from INIT.");
                    }
                }
                else
                {
                    *T1 = 0;
                    *T2 = 0;
                    *T3 = 0;
                    *T4 = 0;
                    if (INFO_LOGS_EN)
                    {
                        LOG_INF("Timestamps expected: %0d, received: %0d", *message_id, msg_id);
                    }
                }
            }
            else
            {
                return FAILURE;
            }
        }
        else // POLL
        {
            dw1000_subread_u32(RX_BUFFER, 0x00, &buffer);

            dest_id = (buffer >> 8) & 0xFF;

            if (dest_id == my_id)
            {
                msg_id = buffer & 0xFF;
                *message_id = msg_id;

                if (INFO_LOGS_EN)
                {
                    LOG_INF("POLL %0d received from INIT.", msg_id);
                }

                *T2 = get_rx_timestamp();
                *T3 = send_resp_message(my_id, TAG_ID, msg_id);

                if (INFO_LOGS_EN)
                {
                    LOG_INF("Response %0d sent drom RESP.", msg_id);
                }
            }
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

double compute_distance_meters(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4)
{
    uint64_t round_trip_time = T4 - T1;
    uint64_t reply_time = T3 - T2;

    double tof_dtu = (double)(round_trip_time - reply_time) / 2.0;
    double tof_sec = tof_dtu * DWT_TIME_UNITS;
    return tof_sec * SPEED_OF_LIGHT;
}

int send_distance(uint8_t src_id, uint32_t distance)
{
    dw1000_subwrite_u32(TX_BUFFER, 0x00, distance);      // 4 bytes
    dw1000_subwrite_u32(TX_BUFFER, 0x04, src_id);        // 1 byte
    dw1000_subwrite_u32(TX_BUFFER, 0x05, DIST_MSG_TYPE); // 1 byte

    new_set_txfctrl(6); // 6 bytes

    new_tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & (SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR)));

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
