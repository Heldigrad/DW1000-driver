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
    // Clear RX buffer
    dw1000_write_u64(RX_BUFFER, 0x00);
    uint32_t status_reg;
    rx_enable(0);

    uint64_t start_time = dw1000_read_sys_time();
    uint64_t now = dw1000_read_sys_time();

    do
    {
        now = dw1000_read_sys_time();
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while ((!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) || (!(has_1_second_passed(start_time, now))));

    if ((!(status_reg & SYS_STATUS_ALL_RX_ERR)) && (status_reg & SYS_STATUS_RXFCG))
    {
        *timestamp = get_rx_timestamp();
        // LOG_INF("RX success! T2 = %08llX", *timestamp);
        dw1000_subread_u64(RX_BUFFER, 0x00, buffer);

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);

        return SUCCESS;
    }
    // LOG_INF("Errors encountered!");
    // print_enabled_bits(status_reg);

    /* Clear RX error events in the DW1000 status register. */
    dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR);
    rx_soft_reset();

    return FAILURE;
}

int transmit(uint64_t data, int len, uint64_t *timestamp)
{
    dw1000_subwrite_u64(TX_BUFFER, 0x00, data);

    set_txfctrl(len);

    tx_start(0);

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
        LOG_ERR_IF_ENABLED("Errors encountered!");
        print_enabled_bits(status);

        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return FAILURE;
    }
}

uint64_t send_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id)
{
    uint64_t tx_timestamp;

    dw1000_subwrite_u8(TX_BUFFER, 0x00, message_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x01, dest_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x02, src_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x03, POLL1_MSG_TYPE);

    set_txfctrl(4);

    tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & (SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR)));

    if (!(status & SYS_STATUS_ALL_TX_ERR))
    {
        tx_timestamp = get_tx_timestamp();
        // LOG_INF("TX success! T1 = %09llX", tx_timestamp);

        /* Clear TX frame sent event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK);

        // LOG_INF("T1 = %0llX", tx_timestamp);

        return tx_timestamp;
    }
    else
    {
        LOG_ERR_IF_ENABLED("Errors encountered!");
        print_enabled_bits(status);

        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return -1;
    }
    LOG_DBG_IF_ENABLED("Sent POLL1.");
}

uint64_t send_resp1_message(uint8_t src_id, uint64_t T2, uint8_t message_id)
{
    uint64_t tx_timestamp;

    dw1000_subwrite_u8(TX_BUFFER, 0x00, message_id);
    dw1000_subwrite_u64(TX_BUFFER, 0x01, T2);
    dw1000_subwrite_u8(TX_BUFFER, 0x06, src_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x07, RESP1_MSG_TYPE);

    set_txfctrl(8);

    tx_start(0);

    uint32_t status;
    do
    {
        dw1000_read_u32(SYS_STATUS, &status);
    } while (!(status & (SYS_STATUS_TXFRS | SYS_STATUS_ALL_TX_ERR)));

    if (!(status & SYS_STATUS_ALL_TX_ERR))
    {
        tx_timestamp = get_tx_timestamp();

        /* Clear TX frame sent event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK);

        return tx_timestamp;
    }
    else
    {
        // LOG_INF("Errors encountered!");
        // print_enabled_bits(status);

        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return 0;
    }
}

uint64_t send_poll2_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id)
{
    uint64_t tx_timestamp;
    uint32_t poll = ((uint32_t)POLL2_MSG_TYPE << 24) |
                    ((uint32_t)src_id << 16) |
                    ((uint32_t)dest_id << 8) |
                    (uint32_t)message_id;

    transmit(poll, 4, &tx_timestamp);

    LOG_DBG_IF_ENABLED("Sent POLL2.");

    return tx_timestamp;
}

int send_resp2_message(uint8_t src_id, uint64_t T3, uint64_t T6, uint8_t message_id)
{
    dw1000_subwrite_u8(TX_BUFFER, 0x00, message_id);
    dw1000_subwrite_u64(TX_BUFFER, 0x01, T6);
    dw1000_subwrite_u64(TX_BUFFER, 0x06, T3);
    dw1000_subwrite_u8(TX_BUFFER, 0x0B, src_id);
    dw1000_subwrite_u8(TX_BUFFER, 0x0C, RESP2_MSG_TYPE);

    set_txfctrl(13);

    tx_start(0);

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
        // LOG_INF("Errors encountered!");
        // print_enabled_bits(status);

        /* Clear TX error event. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_TX_OK | SYS_STATUS_ALL_TX_ERR);

        return FAILURE;
    }
}

int get_poll1_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp)
{
    int res;
    uint8_t frame_len;
    uint64_t message = 0;

    res = receive(&message, timestamp);

    frame_len = get_rx_frame_len(); // Check frame length -> should be 4
    if (frame_len != 4)
    {
        return FAILURE;
    }

    if (res == SUCCESS)
    {
        uint8_t msg_type = (message >> 24) & 0xFF;
        *message_id = (message & 0xFF);
        return (msg_type == POLL1_MSG_TYPE);
    }
    return res;
}

int get_resp1_message(uint8_t src_id, uint8_t dest_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T2)
{
    int res;
    uint8_t frame_len;
    uint64_t message = 0;

    res = receive(&message, timestamp);

    frame_len = get_rx_frame_len(); // Check frame length -> should be 7
    if (frame_len != 7)
    {
        return FAILURE;
    }

    if (res == SUCCESS)
    {
        uint8_t msg_type = (message >> 56) & 0xFF;
        *T2 = (message >> 8) & 0xFFFFFFFFFFULL; // 40-bit mask
        uint8_t msg_id = (message & 0xFF);
        return (msg_type == RESP1_MSG_TYPE && msg_id == message_id);
    }
    return res;
}

int get_poll2_message(uint8_t src_id, uint8_t dest_id, uint8_t *message_id, uint64_t *timestamp)
{
    int res;
    uint8_t frame_len;
    uint64_t message = 0;

    res = receive(&message, timestamp);

    frame_len = get_rx_frame_len(); // Check frame length -> should be 4
    if (frame_len != 4)
    {
        return FAILURE;
    }

    if (res == SUCCESS)
    {
        uint8_t msg_type = (message >> 24) & 0xFF;
        *message_id = (message & 0xFF);
        return (msg_type == POLL2_MSG_TYPE);
    }
    return res;
}

int get_resp2_message(uint8_t src_id, uint8_t message_id, uint64_t *timestamp, uint64_t *T3, uint64_t *T6)
{
    uint8_t frame_len, msg_id, source_id, msg_type;

    uint32_t status_reg;
    rx_enable(0);

    uint64_t start_time = dw1000_read_sys_time();
    uint64_t now = dw1000_read_sys_time();

    do
    {
        now = dw1000_read_sys_time();
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while ((!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) || (!(has_1_second_passed(start_time, now))));

    if ((!(status_reg & SYS_STATUS_ALL_RX_ERR)) && (status_reg & SYS_STATUS_RXFCG))
    {
        frame_len = get_rx_frame_len(); // Check frame length -> should be 12

        if (frame_len != 12)
        {
            return FAILURE;
        }

        *timestamp = get_rx_timestamp();
        // LOG_INF("RX success! T2 = %08llX", *timestamp);
        dw1000_subread_u8(RX_BUFFER, 0x00, &msg_id);
        dw1000_subread_u40(RX_BUFFER, 0x01, T6);
        dw1000_subread_u40(RX_BUFFER, 0x06, T3);
        dw1000_subread_u8(RX_BUFFER, 0x0B, &source_id);
        dw1000_subread_u8(RX_BUFFER, 0x0C, &msg_type);

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);

        return (source_id == src_id && msg_id == message_id && msg_type == RESP2_MSG_TYPE);
    }
    // LOG_INF("Errors encountered!");
    // print_enabled_bits(status_reg);

    /* Clear RX error events in the DW1000 status register. */
    dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR);
    rx_soft_reset();

    return FAILURE;
}

uint8_t get_rx_frame_len()
{
    uint8_t frame_len;
    dw1000_subread_u8(RX_FINFO, 0x00, &frame_len);
    return frame_len & 0x7F;
}

void get_msg_from_init(uint8_t my_id, uint64_t *T2, uint64_t *T3, uint64_t *T6, uint8_t *message_id)
{
    uint32_t status_reg;
    uint8_t msg_type, msg_id, src_id, dest_id;

    rx_enable(0);

    uint64_t start_time = dw1000_read_sys_time();
    uint64_t now = dw1000_read_sys_time();

    do
    {
        now = dw1000_read_sys_time();
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while ((!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) || (!(has_1_second_passed(start_time, now))));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG)) // received successfully
    {
        // Message stucture is the same for POLL1 and POLL2
        dw1000_subread_u8(RX_BUFFER, 0x03, &msg_type);
        dw1000_subread_u8(RX_BUFFER, 0x02, &src_id);
        dw1000_subread_u8(RX_BUFFER, 0x01, &dest_id);
        dw1000_subread_u8(RX_BUFFER, 0x00, &msg_id);

        if (dest_id != my_id)
        {
            LOG_ERR_IF_ENABLED("The message wasn't meant for me!");
            *T2 = 0;
            *T3 = 0;
            *T6 = 0;

            dw1000_subwrite_u40(RX_TIME, 0x00, 0x00);
            dw1000_subwrite_u40(TX_TIME, 0x00, 0x00);

            return;
        }

        if (msg_type == POLL1_MSG_TYPE)
        {
            LOG_DBG_IF_ENABLED("Got POLL1!");
            *T6 = 0;
            *message_id = msg_id;
            *T2 = get_rx_timestamp();
            LOG_DBG_IF_ENABLED("T2 = %0llu", *T2);
            k_msleep(5);
            *T3 = send_resp1_message(my_id, *T2, *message_id);
        }
        else if (msg_type == POLL2_MSG_TYPE)
        {
            LOG_DBG_IF_ENABLED("Got POLL2!");
            if (msg_id != *message_id)
            {
                *T2 = 0;
                *T3 = 0;
                *T6 = 0;

                return;
            }
            *T6 = get_rx_timestamp();
            k_msleep(5);
            send_resp2_message(my_id, *T3, *T6, msg_id);
            LOG_DBG_IF_ENABLED("Sent RESP2!");
        }
    }
}

int get_msg_from_resp(uint8_t destination_id, uint64_t *T2, uint64_t *T3, uint64_t *T4, uint64_t *T5, uint64_t *T6, uint8_t message_id)
{
    uint32_t status_reg;
    uint8_t msg_id, source_id, frame_len;

    rx_enable(0);

    uint64_t start_time = dw1000_read_sys_time();
    uint64_t now = dw1000_read_sys_time();

    do
    {
        now = dw1000_read_sys_time();
        dw1000_read_u32(SYS_STATUS, &status_reg);
    } while ((!(status_reg & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) || (!(has_1_second_passed(start_time, now))));

    if (!(status_reg & SYS_STATUS_ALL_RX_ERR) && (status_reg & SYS_STATUS_RXFCG)) // received successfully
    {

        dw1000_subread_u8(RX_BUFFER, 0x00, &msg_id);
        if (msg_id != message_id)
        {
            LOG_ERR_IF_ENABLED("Message id mismatch!");
            return FAILURE;
        }

        frame_len = get_rx_frame_len() - 2;

        if (frame_len < 10) // RESP1
        {
            LOG_DBG_IF_ENABLED("Got RESP1!");

            uint64_t resp_msg;
            dw1000_subread_u64(RX_BUFFER, 0x00, &resp_msg);

            source_id = (uint8_t)((resp_msg >> 48) & 0xFF);

            if (destination_id != source_id)
            {
                LOG_ERR_IF_ENABLED("RESP1 isn't from the expected anchor!");
                return FAILURE;
            }
            *T4 = get_rx_timestamp();

            dw1000_subread_u40(RX_BUFFER, 0x01, T2);

            k_msleep(5);

            *T5 = send_poll2_message(TAG_ID, destination_id, message_id);
            *T3 = 0;
            *T6 = 0;
        }
        else // RESP2
        {
            LOG_DBG_IF_ENABLED("Got RESP2!");

            dw1000_subread_u8(RX_BUFFER, 0x0B, &source_id);
            if (destination_id != source_id)
            {
                LOG_ERR_IF_ENABLED("RESP2 isn't from the expected anchor!");
                return FAILURE;
            }

            dw1000_subread_u40(RX_BUFFER, 0x01, T6);
            dw1000_subread_u40(RX_BUFFER, 0x06, T3);
        }

        /* Clear good RX frame event in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_RXFCG);

        return SUCCESS;
    }
    else
    {
        LOG_ERR_IF_ENABLED("Timeout or error!");

        /* Clear RX error events in the DW1000 status register. */
        dw1000_write_u32(SYS_STATUS, SYS_STATUS_ALL_RX_ERR);
        rx_soft_reset();

        return FAILURE;
    }
}

uint64_t dw1000_read_sys_time(void)
{
    uint64_t time;
    dw1000_subread_u40(SYS_TIME, 0x00, &time);
    return time;
}

bool has_1_second_passed(uint64_t start_time, uint64_t current_time)
{
    uint64_t elapsed;

    // DW1000 SYS_TIME is 40-bit and wraps around ~17.2 seconds
    if (current_time >= start_time)
    {
        elapsed = current_time - start_time;
    }
    else
    {
        // handle 40-bit wraparound
        elapsed = (0xFFFFFFFFFFULL - start_time) + current_time + 1;
    }

    return elapsed >= ONE_SECOND_TICKS;
}

double compute_ds_twr_distance_basic(uint64_t T1, uint64_t T2, uint64_t T3, uint64_t T4, uint64_t T5, uint64_t T6)
{
    int64_t Tround1 = ((int64_t)T4 - (int64_t)T1);
    int64_t Treply1 = ((int64_t)T3 - (int64_t)T2);
    int64_t Treply2 = ((int64_t)T5 - (int64_t)T4);
    int64_t Tround2 = ((int64_t)T6 - (int64_t)T3);

    int64_t numerator = (int64_t)(Tround1 * Tround2) - (int64_t)(Treply2 * Treply1);
    int64_t denominator = Tround1 + Tround2 + Treply1 + Treply2;

    if (denominator == 0)
        return -1.0;

    double tof_ticks = (double)numerator / (double)denominator;
    double tof_seconds = tof_ticks * DWT_TIME_UNITS;
    double distance = tof_seconds * SPEED_OF_LIGHT;

    return distance;
}

double get_coord(double B1, double B2, double DO, double D1, double D2)
{
    double cosA = (DO * DO + B1 * B1 - D1 * D1) / (2 * B1 * DO);
    double cosB = (DO * DO + B2 * B2 - D2 * D2) / (2 * B2 * DO);
    double cosC = sqrt(1 - cosA * cosA - cosB * cosB);

    return cosC * DO;
}

void compute_coord(double Distances[4])
{
    double DO = Distances[0];
    double DX = Distances[1];
    double DY = Distances[2];
    double DZ = Distances[3];

    double X = get_coord(DIST_Z, DIST_Y, DO, DZ, DY);
    double Y = get_coord(DIST_X, DIST_Z, DO, DX, DZ);
    double Z = get_coord(DIST_X, DIST_Y, DO, DX, DY);

    LOG_INF("P = (%0f, %0f, %0f)", X, Y, Z);
}

const int antenna_delays[4] = {16343, 16379, 16347, 16343};

void set_antenna_delay(int anchor_id)
{
    set_rx_antenna_delay(antenna_delays[anchor_id - 1]);
    set_tx_antenna_delay(antenna_delays[anchor_id - 1]);
}