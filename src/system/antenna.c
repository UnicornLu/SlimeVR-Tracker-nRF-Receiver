/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/
#include "antenna.h"

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(antenna, LOG_LEVEL_INF);

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, ant_set_gpios)
static const struct gpio_dt_spec ant_set = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, ant_set_gpios);
#define ANT_SET_AVAILABLE 1
#else
#define ANT_SET_AVAILABLE 0
#pragma message "ANT-SET GPIO not defined in device tree"
#endif

#if DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, csd_gpios)
static const struct gpio_dt_spec csd = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, csd_gpios);
#define CSD_AVAILABLE 1
#else
#define CSD_AVAILABLE 0
#pragma message "CSD GPIO not defined in device tree"
#endif

static uint8_t current_antenna = 0; // 0 = antenna 0, 1 = antenna 1
static int64_t last_switch_time = 0;
#define ANTENNA_SWITCH_INTERVAL_MS 1000 // Switch antenna every 1 second for diversity

int antenna_init(void)
{
	int err = 0;

#if ANT_SET_AVAILABLE
	if (!device_is_ready(ant_set.port)) {
		LOG_ERR("ANT-SET GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&ant_set, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure ANT-SET GPIO: %d", err);
		return err;
	}
	LOG_INF("ANT-SET GPIO initialized on pin %d", ant_set.pin);
#else
	LOG_WRN("ANT-SET GPIO not available");
#endif

#if CSD_AVAILABLE
	if (!device_is_ready(csd.port)) {
		LOG_ERR("CSD GPIO device not ready");
		return -ENODEV;
	}

	err = gpio_pin_configure_dt(&csd, GPIO_OUTPUT_INACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure CSD GPIO: %d", err);
		return err;
	}
	LOG_INF("CSD GPIO initialized on pin %d", csd.pin);
#else
	LOG_WRN("CSD GPIO not available");
#endif

	current_antenna = 0;
	last_switch_time = k_uptime_get();

	return 0;
}

void antenna_select_0(void)
{
#if ANT_SET_AVAILABLE
	gpio_pin_set_dt(&ant_set, 0);
	current_antenna = 0;
	LOG_DBG("Switched to antenna 0");
#endif
}

void antenna_select_1(void)
{
#if ANT_SET_AVAILABLE
	gpio_pin_set_dt(&ant_set, 1);
	current_antenna = 1;
	LOG_DBG("Switched to antenna 1");
#endif
}

void antenna_toggle(void)
{
#if ANT_SET_AVAILABLE
	current_antenna = !current_antenna;
	gpio_pin_set_dt(&ant_set, current_antenna);
	LOG_DBG("Toggled to antenna %d", current_antenna);
#endif
}

uint8_t antenna_get_current(void)
{
	return current_antenna;
}

void antenna_periodic_switch(void)
{
#if ANT_SET_AVAILABLE
	int64_t now = k_uptime_get();
	if (now - last_switch_time >= ANTENNA_SWITCH_INTERVAL_MS) {
		antenna_toggle();
		last_switch_time = now;
	}
#endif
}

