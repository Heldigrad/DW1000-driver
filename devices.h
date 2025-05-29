#pragma once
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "defines.h"

// SPI device and configuration
extern const struct device *spi_dev;
extern const struct spi_config spi_cfg;

extern struct spi_dt_spec spispec;

// GPIO for Chip Select (CS) and reset
extern const struct gpio_dt_spec cs_gpio;
extern const struct gpio_dt_spec reset_gpio;

int check_devices_ready();

void reset_devices();