#pragma once

#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "defines.h"

LOG_MODULE_DECLARE(dw1000_spi, LOG_LEVEL_DBG);