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
        LOG_INF("Errors encountered!");
        print_enabled_bits(status_reg);

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

    // Time reply duration
    Treply = T3 - T2;

    // One-way time of flight
    Tprop = (Tround - Treply) / 2;

    // Convert to seconds using DW1000 time units
    double tof_sec = Tprop * DWT_TIME_UNITS;

    // Compute distance
    double distance_m = tof_sec * SPEED_OF_LIGHT;

    return distance_m;
}