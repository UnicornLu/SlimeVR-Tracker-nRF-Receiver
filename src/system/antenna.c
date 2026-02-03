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
#include <zephyr/spinlock.h>
#include <string.h>

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
#define ANTENNA_SWITCH_INTERVAL_MS 200 // Evaluate antenna selection every 200 ms
#define ANTENNA_HYSTERESIS_DB 3
#define ANTENNA_LOSS_PENALTY_DIV 10 // 1 dB penalty per 10% loss (permille/10)
#define ANTENNA_STATS_STALE_MS 1000

struct antenna_window_stats {
	int32_t rssi_sum;
	uint32_t rssi_count;
	uint32_t received_packets;
	uint32_t lost_packets;
};

struct antenna_eval_stats {
	int32_t avg_rssi;
	uint32_t loss_rate_permille;
	int64_t last_update_ms;
	bool valid;
};

static struct antenna_window_stats antenna_window[2] = {0};
static struct antenna_eval_stats antenna_eval[2] = {0};
static struct k_spinlock antenna_lock;

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
	memset(antenna_window, 0, sizeof(antenna_window));
	memset(antenna_eval, 0, sizeof(antenna_eval));

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

void antenna_record_rx(int8_t rssi)
{
#if ANT_SET_AVAILABLE
	k_spinlock_key_t key = k_spin_lock(&antenna_lock);
	struct antenna_window_stats *stats = &antenna_window[current_antenna];
	stats->rssi_sum += rssi;
	stats->rssi_count++;
	stats->received_packets++;
	k_spin_unlock(&antenna_lock, key);
#endif
}

void antenna_record_loss(uint32_t lost_packets)
{
#if ANT_SET_AVAILABLE
	if (lost_packets == 0) {
		return;
	}
	k_spinlock_key_t key = k_spin_lock(&antenna_lock);
	antenna_window[current_antenna].lost_packets += lost_packets;
	k_spin_unlock(&antenna_lock, key);
#endif
}

static void antenna_update_eval_stats(int64_t now_ms)
{
	for (int i = 0; i < 2; i++) {
		struct antenna_window_stats *window = &antenna_window[i];
		if (window->rssi_count == 0 && window->lost_packets == 0 && window->received_packets == 0) {
			continue;
		}
		int32_t avg_rssi = 0;
		if (window->rssi_count > 0) {
			avg_rssi = window->rssi_sum / (int32_t)window->rssi_count;
		}
		uint32_t total_packets = window->received_packets + window->lost_packets;
		uint32_t loss_rate_permille = 1000;
		if (total_packets > 0) {
			loss_rate_permille = (window->lost_packets * 1000U) / total_packets;
		}
		antenna_eval[i].avg_rssi = avg_rssi;
		antenna_eval[i].loss_rate_permille = loss_rate_permille;
		antenna_eval[i].last_update_ms = now_ms;
		antenna_eval[i].valid = true;

		memset(window, 0, sizeof(*window));
	}
}

void antenna_periodic_switch(void)
{
#if ANT_SET_AVAILABLE
	int64_t now = k_uptime_get();
	if (now - last_switch_time < ANTENNA_SWITCH_INTERVAL_MS) {
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&antenna_lock);
	antenna_update_eval_stats(now);
	struct antenna_eval_stats eval[2] = {antenna_eval[0], antenna_eval[1]};
	k_spin_unlock(&antenna_lock, key);

	bool stale[2] = {
		!eval[0].valid || (now - eval[0].last_update_ms > ANTENNA_STATS_STALE_MS),
		!eval[1].valid || (now - eval[1].last_update_ms > ANTENNA_STATS_STALE_MS),
	};
	if (stale[0] || stale[1]) {
		uint8_t target = current_antenna;
		if (stale[0] && !stale[1]) {
			target = 0;
		} else if (stale[1] && !stale[0]) {
			target = 1;
		} else {
			target = !current_antenna;
		}
		if (target != current_antenna) {
			if (target == 0) {
				antenna_select_0();
			} else {
				antenna_select_1();
			}
		}
		last_switch_time = now;
		return;
	}

	int32_t score0 = eval[0].avg_rssi - (int32_t)(eval[0].loss_rate_permille / ANTENNA_LOSS_PENALTY_DIV);
	int32_t score1 = eval[1].avg_rssi - (int32_t)(eval[1].loss_rate_permille / ANTENNA_LOSS_PENALTY_DIV);
	int32_t diff = score1 - score0;

	if (diff > ANTENNA_HYSTERESIS_DB && current_antenna != 1) {
		antenna_select_1();
	} else if (diff < -ANTENNA_HYSTERESIS_DB && current_antenna != 0) {
		antenna_select_0();
	}

	last_switch_time = now;
#endif
}
