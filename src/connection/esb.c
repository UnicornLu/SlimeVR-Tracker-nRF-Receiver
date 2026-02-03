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
#include "esb.h"

#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/crc.h>

#include "globals.h"
#include "hid.h"
#include "system/system.h"
#include "system/antenna.h"

#define RADIO_RETRANSMIT_DELAY CONFIG_RADIO_RETRANSMIT_DELAY
#define RADIO_RF_CHANNEL CONFIG_RADIO_RF_CHANNEL

// TDMA parameters (copied from Tracker)
#define TDMA_ENABLED 0
#define TDMA_NUM_TRACKERS 10
#define TDMA_PACKETS_PER_SECOND 111 // Target TPS per tracker
#define TDMA_PACKET_INTERVAL_US (1000000 / TDMA_PACKETS_PER_SECOND)
#define TDMA_SLOT_DURATION_US (TDMA_PACKET_INTERVAL_US / TDMA_NUM_TRACKERS)

static struct esb_payload rx_payload;
// static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);
// static struct esb_payload tx_payload_timer = ESB_CREATE_PAYLOAD(0,
//														  0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
// 0, 0, 0, 0, 0, 0, 0);
static struct esb_payload tx_payload_sync = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0);

// TX statistics
struct tx_stats {
	uint32_t total_success;          // Total successful transmissions
	uint32_t total_failed;           // Total failed transmissions
	uint32_t consecutive_fails;      // Consecutive failed transmissions
	int64_t last_fail_time;          // Last failure timestamp
	int64_t last_log_time;           // Last log timestamp
	uint32_t success_since_last_log; // Successful transmissions since last log
	uint32_t failed_since_last_log;  // Failed transmissions since last log
};
static struct tx_stats tx_statistics = {0};

#define TX_LOG_INTERVAL_MS 1000 // TX statistics log interval (max once per second)

struct pairing_event {
	uint8_t packet[8];
};

// Queue pairing packets from the ISR context to the pairing worker thread.
K_MSGQ_DEFINE(esb_pairing_msgq, sizeof(struct pairing_event), 8, 4);

static K_MUTEX_DEFINE(tracker_store_lock);

static uint8_t last_packet_sequence[MAX_TRACKERS];    // Track the last packet sequence for each tracker
static uint8_t last_ping_counter[MAX_TRACKERS] = {0}; // Track the last PING counter for each tracker
static bool ping_counter_initialized[MAX_TRACKERS]
	= {false};                                      // Flag indicating if a PING has been received from this tracker
static uint64_t last_ping_time[MAX_TRACKERS] = {0}; // Track the last time a PING was received from each tracker
static uint8_t last_pong_queued_counter[MAX_TRACKERS] = {0}; // Track the last PONG counter enqueued for each tracker
static uint8_t packet_count[MAX_TRACKERS] = {0};             // Packet count received from each tracker
static uint8_t tracker_remote_command[MAX_TRACKERS] = {ESB_PONG_FLAG_NORMAL}; // Tracker remote command flag
static uint32_t tracker_channel_value = 0;               // Channel value to be set (for SET_CHANNEL command)
static int16_t pending_sens_data[MAX_TRACKERS][3] = {0}; // Store pending sensitivity data
static uint8_t receiver_rf_channel = 0xFF; // Current RF channel of the receiver, 0xFF indicates using default value

#define PING_TIMEOUT_MS 5000 // PING timeout threshold: 5 seconds

// Channel change confirmation tracking
static bool channel_change_pending = false; // Indicates if a channel change is pending
static uint8_t pending_channel = 0;         // The channel value to switch to
static atomic_t channel_ack_mask
	= ATOMIC_INIT(0);                      // Bitmask to track which trackers have acknowledged the channel change
static int64_t channel_change_timeout = 0; // Timestamp for channel change timeout
#define CHANNEL_CHANGE_TIMEOUT_MS 30000    // Timeout duration for waiting for all trackers to acknowledge

// Packet statistics structure
struct packet_stats {
	uint32_t total_received;    // Total number of packets received (excluding duplicates)
	uint32_t normal_packets;    // Number of packets received in normal sequence
	uint32_t gap_events;        // Number of gap events (potential packet loss)
	uint32_t out_of_order;      // Number of out-of-order packets
	uint32_t duplicate_packets; // Number of duplicate packets
	uint32_t restart_events;    // Number of restart events
	uint32_t total_gaps;        // Total number of gaps (estimated packet loss)
	uint32_t last_sequence;     // Last normal sequence number
	uint64_t last_packet_time;  // Timestamp of the last received packet
	bool first_packet;          // Flag indicating if it's the first packet
	// TPS calculation related
	uint32_t packets_in_last_second; // Number of packets in the last second
	uint64_t last_tps_time;          // Last TPS calculation timestamp
	uint32_t current_tps;            // Current TPS (packets per second)
};

static struct packet_stats tracker_stats[MAX_TRACKERS] = {0};
#define STATS_PRINT_INTERVAL_MS 5000     // Print statistics every 5 seconds
#define TPS_CALCULATION_INTERVAL_MS 1000 // Calculate TPS every second
#define TPS_MONITOR_INTERVAL_MS 500

// TDMA Statistics
struct tdma_stats {
	int64_t sum_offset;
	uint64_t sum_sq_offset;
	int32_t min_offset;
	int32_t max_offset;
	uint32_t count;
	uint32_t violations;
	int64_t last_log_time;
};

static struct tdma_stats g_tdma_stats[MAX_TRACKERS] = {0};

static void esb_stats_thread(void);
K_THREAD_DEFINE(esb_stats_thread_id, 512, esb_stats_thread, NULL, NULL, NULL, 8, 0, 0);

LOG_MODULE_REGISTER(esb_event, LOG_LEVEL_INF);

static void esb_thread(void);
K_THREAD_DEFINE(esb_thread_id, 1024, esb_thread, NULL, NULL, NULL, 7, 0, 0);

static bool esb_parse_pair(const uint8_t packet[8]);

static int check_packet_sequence(uint8_t tracker_id, uint8_t received_seq)
{
	if (tracker_id >= MAX_TRACKERS) {
		return 2;
	}

	struct packet_stats *stats = &tracker_stats[tracker_id];
	uint64_t current_time = k_uptime_get();
	// Update TPS calculation
	if (stats->last_tps_time == 0) {
		stats->last_tps_time = current_time;
		stats->packets_in_last_second = 0;
	} else if (current_time - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
		// Calculate TPS and reset counter
		stats->current_tps = stats->packets_in_last_second;
		stats->packets_in_last_second = 0;
		stats->last_tps_time = current_time;
	}

	// Each packet counts towards TPS calculation
	stats->packets_in_last_second++;
	stats->last_packet_time = current_time;

	// First packet, accept directly
	if (packet_count[tracker_id] == 0) {
		// Update received packet count
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id] = 1;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		stats->first_packet = false;
		LOG_DBG("First packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	uint8_t last_seq = last_packet_sequence[tracker_id];
	uint8_t expected_seq = (last_seq + 1) & 0xFF;

	LOG_DBG(
		"Packet check: tracker=%d, received=%d, last=%d, expected=%d",
		tracker_id,
		received_seq,
		last_seq,
		expected_seq
	);
	// Normal next sequence number
	if (received_seq == expected_seq) {
		// Update received packet count
		stats->total_received++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->normal_packets++;
		stats->last_sequence = received_seq;
		LOG_DBG("Normal packet: tracker=%d, seq=%d", tracker_id, received_seq);
		return 0;
	}

	// Calculate sequence difference (considering wrap-around)
	uint8_t diff_forward = (received_seq - last_seq) & 0xFF;  // Forward difference (including wrap-around)
	uint8_t diff_backward = (last_seq - received_seq) & 0xFF; // Backward difference (including wrap-around)

	if (diff_forward == 0) {
		// Same sequence number - this is a true duplicate packet
		stats->duplicate_packets++;
		return 4; // Duplicate packet
	}

	// Backward jump (old packet) is considered out-of-order, keep current window unchanged
	// Note: Out-of-order packets are not counted in total_received as they are dropped and not forwarded to the
	// application layer Check condition: small diff_backward (< 64) indicates a true backward jump (old packet)
	if (diff_backward > 0 && diff_backward < 64) {
		stats->out_of_order++;
		// Log single out-of-order event only at DEBUG level to reduce output
		LOG_DBG(
			"Out-of-order packet dropped: tracker=%d, seq=%d (expected >%d), "
			"backward=%d",
			tracker_id,
			received_seq,
			last_seq,
			diff_backward
		);
		return 2;
	}

	// Detect large jump (possibly a restart)
	// If the jump exceeds 80, it's likely a tracker restart or wrap-around
	// In this case, a large number of gaps should not be accumulated
	if (diff_forward > 80) {
		// This is a large jump, treated as a tracker restart
		stats->total_received++;
		stats->restart_events++;
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->last_sequence = received_seq;
		// Restart events are kept at WARNING level because this is important information
		LOG_WRN(
			"Tracker restart detected: tracker=%d, last_seq=%d, new_seq=%d, jump=%d",
			tracker_id,
			last_seq,
			received_seq,
			diff_forward
		);
		return 3; // Restart event
	}

	// Forward jump (normal packet loss range)
	if (diff_forward > 0 && diff_forward <= 80) {
		stats->total_received++;
		stats->gap_events++;
		stats->total_gaps += (diff_forward - 1); // Estimate number of lost packets
		last_packet_sequence[tracker_id] = received_seq;
		packet_count[tracker_id]++;
		stats->last_sequence = received_seq;
		// Log single gap only at DEBUG level to reduce output (summary statistics will show total gaps)
		LOG_DBG(
			"Gap detected: tracker=%d, seq=%d, gap=%d (forward=%d)",
			tracker_id,
			received_seq,
			diff_forward - 1,
			diff_forward
		);
		return 1;
	}

	// Default case, should not be reached; treat as duplicate for safety
	stats->duplicate_packets++;
	return 4;
}

// Prints statistics for a specific tracker
// Prints statistics for a single tracker
static void print_tracker_stats(uint8_t tracker_id)
{
	struct packet_stats *stats = &tracker_stats[tracker_id];

	if (stats->total_received == 0 && stats->duplicate_packets == 0) {
		return;
	}

	// Total received packets (including duplicates and out-of-order)
	uint32_t total_receives = stats->total_received + stats->duplicate_packets + stats->out_of_order;

	// Calculate various rates (using permille to avoid floating-point operations)
	uint32_t duplicate_rate = 0;
	uint32_t out_of_order_rate = 0;

	if (total_receives > 0) {
		duplicate_rate = (stats->duplicate_packets * 1000) / total_receives;
		out_of_order_rate = (stats->out_of_order * 1000) / total_receives;
	}

	// Estimate packet loss rate: based on total gaps and received packets
	uint32_t estimated_sent = stats->total_received + stats->total_gaps;
	uint32_t estimated_loss_rate = 0;
	if (estimated_sent > 0) {
		estimated_loss_rate = (stats->total_gaps * 1000) / estimated_sent;
	}

	LOG_INF(
		"Tracker %d: Recv=%u(+%u dup +%u ooo), Normal=%u, EstLoss=%u.%u%% (%u gaps), "
		"Dup=%u.%u%%, OOO=%u.%u%%, Restart=%u, TPS=%u",
		tracker_id,
		stats->total_received,
		stats->duplicate_packets,
		stats->out_of_order,
		stats->normal_packets,
		estimated_loss_rate / 10,
		estimated_loss_rate % 10,
		stats->total_gaps,
		duplicate_rate / 10,
		duplicate_rate % 10,
		out_of_order_rate / 10,
		out_of_order_rate % 10,
		stats->restart_events,
		stats->current_tps
	);
}

static void print_tracker_stats_batch(void)
{
	for (int i = 0; i < MAX_TRACKERS; i++) {
		if (tracker_stats[i].total_received > 0 || tracker_stats[i].duplicate_packets > 0) {
			print_tracker_stats(i);
		}
	}
}

static void esb_stats_thread(void)
{
	int64_t last_log_time = k_uptime_get();

	while (1) {
		k_msleep(TPS_MONITOR_INTERVAL_MS);

		uint64_t now = (uint64_t)k_uptime_get();
		for (int i = 0; i < MAX_TRACKERS; i++) {
			struct packet_stats *stats = &tracker_stats[i];
			if (stats->last_tps_time == 0) {
				continue;
			}
			if (now - stats->last_tps_time >= TPS_CALCULATION_INTERVAL_MS) {
				if (stats->packets_in_last_second > 0) {
					stats->current_tps = stats->packets_in_last_second;
				} else if (stats->last_packet_time && now - stats->last_packet_time >= TPS_CALCULATION_INTERVAL_MS) {
					stats->current_tps = 0;
				}
				stats->packets_in_last_second = 0;
				stats->last_tps_time = now;
			}
		}

		if (now - last_log_time >= STATS_PRINT_INTERVAL_MS) {
			bool has_data = false;
			for (int i = 0; i < MAX_TRACKERS; i++) {
				if (tracker_stats[i].total_received > 0) {
					has_data = true;
					break;
				}
			}

			if (has_data) {
				LOG_INF("=== Packet Statistics ===");
				print_tracker_stats_batch();
			}
			last_log_time = now;
		}
	}
}

void event_handler(struct esb_evt const *event)
{
	int64_t now = k_uptime_get();

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		tx_statistics.total_success++;
		tx_statistics.success_since_last_log++;
		tx_statistics.consecutive_fails = 0; // Reset consecutive fail counter

		break;

	case ESB_EVENT_TX_FAILED:
		tx_statistics.total_failed++;
		tx_statistics.failed_since_last_log++;
		tx_statistics.consecutive_fails++;
		tx_statistics.last_fail_time = now;
		// Only output detailed logs when there are many consecutive failures
		if (tx_statistics.consecutive_fails <= 3) {
			// Low frequency failures - DEBUG level
			LOG_DBG("TX FAILED (attempts=%u, consecutive=%u)", event->tx_attempts, tx_statistics.consecutive_fails);
		} else if (tx_statistics.consecutive_fails <= 10) {
			// Medium frequency failures - log every 5 times
			if (tx_statistics.consecutive_fails % 5 == 0) {
				LOG_WRN(
					"TX FAILED repeatedly! (consecutive=%u, attempts=%u)",
					tx_statistics.consecutive_fails,
					event->tx_attempts
				);
			}
		} else {
			// High frequency failures - log every 10 times
			if (tx_statistics.consecutive_fails % 10 == 0) {
				LOG_ERR("TX CRITICAL: %u consecutive failures! Check RF environment", tx_statistics.consecutive_fails);
			}

			// Recovery mechanism
			if (tx_statistics.consecutive_fails >= 50) {
				LOG_ERR("Too many TX failures, attempting ESB recovery...");
				esb_flush_tx();
				k_msleep(10);
				tx_statistics.consecutive_fails = 0; // Reset to avoid spam
			}
		}
		break;
	case ESB_EVENT_RX_RECEIVED: {
		int err = 0;
		while (!err) {
			err = esb_read_rx_payload(&rx_payload);
			uint32_t current_rx_ticks = k_uptime_ticks();
			uint64_t current_rx_us = k_ticks_to_us_floor64(current_rx_ticks);
			if (err == -ENODATA) {
				break;
			} else if (err) {
				LOG_ERR("Error while reading rx packet: %d", err);
				break;
			}
			switch (rx_payload.length) {
			case 1: // ACK packet
				LOG_DBG("RX ACK len=%u pipe=%u data=%02X", rx_payload.length, rx_payload.pipe, rx_payload.data[0]);
				break;
			case 8: {
				struct pairing_event evt = {0};
				memcpy(evt.packet, rx_payload.data, sizeof(evt.packet));
				LOG_DBG("rx: %16llX", *(uint64_t *)evt.packet);
				int q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
				if (q_err) {
					struct pairing_event discarded;
					if (k_msgq_get(&esb_pairing_msgq, &discarded, K_NO_WAIT) == 0) {
						q_err = k_msgq_put(&esb_pairing_msgq, &evt, K_NO_WAIT);
					}
					if (q_err) {
						LOG_WRN("ACK queue full, dropping packet type %u", evt.packet[1]);
					}
				}
				switch (evt.packet[1]) {
				case 1: // receives ack generated from last packet
					LOG_DBG("RX Pairing Sent ACK");
					break;
				case 2: // should "acknowledge" pairing data sent from
						// receiver
					LOG_DBG("RX Pairing ACK Receiver");
					break;
				case 0:
					LOG_INF("RX Pairing Request");
					break;
				default:
					LOG_WRN("Unexpected pairing packet type %u", evt.packet[1]);
					break;
				}
				break;
			} break;
			case ESB_PING_LEN: {
				LOG_DBG(
					"Received PING type=%u id=%u ctr=%u",
					rx_payload.data[0],
					rx_payload.data[1],
					rx_payload.data[2]
				);

				// TDMA Slot Check for PING
				uint8_t tracker_id = rx_payload.data[1];

				// Parse new PING fields (added in protocol update)
				uint32_t expected_rx_ticks = ((uint32_t)rx_payload.data[3] << 24) | ((uint32_t)rx_payload.data[4] << 16)
										   | ((uint32_t)rx_payload.data[5] << 8) | ((uint32_t)rx_payload.data[6]);

				if (tracker_id < MAX_TRACKERS) {
					// Update stats
					struct tdma_stats *stats = &g_tdma_stats[tracker_id];

					// Diagnostic: Compare expected vs actual receiver time
					int32_t rx_time_diff_ticks = (int32_t)(current_rx_ticks - expected_rx_ticks);
					uint64_t rx_time_diff_us
						= k_ticks_to_us_floor64((rx_time_diff_ticks < 0 ? -rx_time_diff_ticks : rx_time_diff_ticks));

					// Log periodically
					int64_t now_ms = k_uptime_get();
					if (stats->count > 0) {
						int64_t mean = stats->sum_offset / stats->count;
						uint64_t variance = (stats->sum_sq_offset / stats->count) - (mean * mean);
						uint32_t std_dev = 0;
						if (variance > 0) {
							uint64_t s = variance / 2;
							if (s > 0) {
								uint64_t x = s;
								uint64_t y = (x + variance / x) / 2;
								while (y < x) {
									x = y;
									y = (x + variance / x) / 2;
								}
								std_dev = (uint32_t)x;
							} else {
								std_dev = (uint32_t)variance;
							}
						}

#if TDMA_ENABLED
						if (stats->violations > 3) {
							LOG_WRN(
								"TDMA Stats ID=%u Count=%u Viol=%u Mean=%lld us StdDev=%u us Range=[%d, %d] "
								"RxTimeDiff=%s%llu us",
								tracker_id,
								stats->count,
								stats->violations,
								mean,
								std_dev,
								stats->min_offset,
								stats->max_offset,
								rx_time_diff_ticks >= 0 ? "+" : "-",
								rx_time_diff_us
							);
						} else {
							LOG_DBG(
								"TDMA Stats ID=%u Count=%u Viol=%u Mean=%lld us StdDev=%u us Range=[%d, %d] "
								"RxTimeDiff=%s%llu us",
								tracker_id,
								stats->count,
								stats->violations,
								mean,
								std_dev,
								stats->min_offset,
								stats->max_offset,
								rx_time_diff_ticks >= 0 ? "+" : "-",
								rx_time_diff_us
							);
						}
#endif
					}

					// Reset stats
					memset(stats, 0, sizeof(struct tdma_stats));
					stats->last_log_time = now_ms;
				}

				// Check for PING control packet and respond with PONG
				if (rx_payload.data[0] == ESB_PING_TYPE) {
					uint8_t tracker_id = rx_payload.data[1];
					uint8_t counter = rx_payload.data[2];

					uint8_t ping_ack_flag = rx_payload.data[7];

					if (rx_payload.pipe != 1 + (tracker_id % 7)) {
						static uint8_t pipe_mismatch_count[MAX_TRACKERS] = {0};
						pipe_mismatch_count[tracker_id]++;

						if (pipe_mismatch_count[tracker_id] % 10 == 1) {
							LOG_WRN(
								"PING pipe mismatch (x%u): id=%u, expected "
								"pipe=%u got pipe=%u",
								pipe_mismatch_count[tracker_id],
								tracker_id,
								1 + (tracker_id % 7),
								rx_payload.pipe
							);
						} else {
							LOG_DBG(
								"PING pipe mismatch: id=%u, expected pipe=%u "
								"got pipe=%u",
								tracker_id,
								1 + (tracker_id % 7),
								rx_payload.pipe
							);
						}
					}

					// check crc for PING
					uint8_t crc_calc = crc8_ccitt(0x07, rx_payload.data, ESB_PING_LEN - 1);
					if (rx_payload.data[ESB_PING_LEN - 1] != crc_calc) {
						// CRC error - only log warning if consecutive errors occur
						static uint8_t crc_error_count[MAX_TRACKERS] = {0};
						crc_error_count[tracker_id]++;

						if (crc_error_count[tracker_id] % 5 == 1) {
							// Log every 5th error
							LOG_WRN(
								"PING CRC mismatch (x%u): id=%u expected %02X "
								"got %02X",
								crc_error_count[tracker_id],
								tracker_id,
								crc_calc,
								rx_payload.data[ESB_PING_LEN - 1]
							);
						} else {
							LOG_DBG(
								"PING CRC mismatch: id=%u expected %02X got "
								"%02X",
								tracker_id,
								crc_calc,
								rx_payload.data[ESB_PING_LEN - 1]
							);
						}
						break;
					}

					uint32_t tracker_estimated_server_ticks
						= ((uint32_t)rx_payload.data[3] << 24) | ((uint32_t)rx_payload.data[4] << 16)
						| ((uint32_t)rx_payload.data[5] << 8) | ((uint32_t)rx_payload.data[6]);
					// Calculate signed ticks difference
					int32_t ticks_diff = (int32_t)current_rx_ticks - (int32_t)tracker_estimated_server_ticks;
					// Get absolute value for conversion to microseconds
					uint32_t ticks_diff_abs = (uint32_t)(ticks_diff < 0 ? -ticks_diff : ticks_diff);
					LOG_DBG(
						"Tracker %u PING ctr=%u ticks_offset=%s%d us",
						tracker_id,
						counter,
						ticks_diff >= 0 ? "+" : "-",          // Add sign to the log
						k_ticks_to_us_floor32(ticks_diff_abs) // Convert absolute tick difference to microseconds
					);

					// First PING received for this tracker - accept directly, no sequence check
					if (!ping_counter_initialized[tracker_id]) {
						ping_counter_initialized[tracker_id] = true;
						last_ping_counter[tracker_id] = counter;
						last_ping_time[tracker_id] = k_uptime_get();
						last_pong_queued_counter[tracker_id] = 0xFF; // Mark as not queued
						LOG_DBG("First PING from tracker %u, ctr=%u, initializing", tracker_id, counter);
						// Continue processing, send PONG
					} else {
						// Already initialized, perform sequence check
						uint64_t current_time = k_uptime_get();
						bool is_duplicate = false;
						bool is_out_of_order = false;
						bool is_large_gap = false;
						bool is_tracker_restart = false;

						// Check for timeout - if no PING received for more than 5 seconds, reset expectation
						if (last_ping_time[tracker_id] > 0
							&& (current_time - last_ping_time[tracker_id]) > PING_TIMEOUT_MS) {
							LOG_INF(
								"PING timeout (%llu ms), resetting tracker %u counter tracking",
								current_time - last_ping_time[tracker_id],
								tracker_id
							);
							last_ping_counter[tracker_id] = counter;
							last_ping_time[tracker_id] = current_time;
							last_pong_queued_counter[tracker_id] = 0xFF;
							// Continue processing this PING
						} else {
							// Calculate difference
							int counter_diff = (int)counter - (int)last_ping_counter[tracker_id];
							if (counter_diff < 0) {
								counter_diff += 256; // Handle wrap-around
							}

							if (counter_diff == 0) {
								// Duplicate packet
								is_duplicate = true;
							} else if (counter_diff >= 1 && counter_diff <= 100) {
								// Normal progression (possible packet loss)
								if (counter_diff > 5) {
									is_large_gap = true;
								}
							} else if (counter_diff > 128) {
								// Possibly backward or wrap-around
								int backward_amount = 256 - counter_diff;

								if (backward_amount <= 10) {
									// Small backward step - truly out of order
									is_out_of_order = true;
								} else if (counter < 5) {
									// Large backward step and small counter - possibly a restart
									is_tracker_restart = true;
									LOG_INF(
										"Tracker restart detected: id=%u old_ctr=%u new_ctr=%u (backward=%d)",
										tracker_id,
										last_ping_counter[tracker_id],
										counter,
										backward_amount
									);
								} else {
									// Large backward step but counter not small - long packet loss + wrap-around
									is_large_gap = true;
									LOG_WRN(
										"Long packet loss with wraparound: id=%u last=%u new=%u (backward=%d, "
										"accepting)",
										tracker_id,
										last_ping_counter[tracker_id],
										counter,
										backward_amount
									);
								}
							} else {
								// counter_diff is between 101-128 - large packet loss
								is_large_gap = true;
							}

							// Update timestamp
							last_ping_time[tracker_id] = current_time;
						}

						// Handle various cases
						if (is_tracker_restart) {
							// Tracker restarted, reset counter tracking
							last_ping_counter[tracker_id] = counter;
							// Reset PONG queue tracking
							last_pong_queued_counter[tracker_id] = 0xFF;
							// Continue processing this PING, send PONG
						} else if (is_duplicate) {
							// Same counter as last time - likely a retransmit
							LOG_DBG("Duplicate PING detected: id=%u ctr=%u (retransmit or retry)", tracker_id, counter);

							// Check if we already queued a PONG for this counter
							if (counter == last_pong_queued_counter[tracker_id]) {
								LOG_DBG("PONG already queued for ctr=%u, skipping duplicate queue", counter);
								// Don't queue again, tracker will get PONG on next packet
								break;
							}
						} else if (is_out_of_order) {
							// Out-of-order PING - this is an old packet that arrived late
							// Don't process it to avoid sending stale PONG
							int backward_amount = 256 - ((int)counter - (int)last_ping_counter[tracker_id] + 256) % 256;
							LOG_WRN(
								"Out-of-order PING: id=%u ctr=%u (expected >%u, -%d backward), SKIPPING",
								tracker_id,
								counter,
								last_ping_counter[tracker_id],
								backward_amount
							);
							// Don't update last_ping_counter, don't queue PONG
							break;
						} else if (is_large_gap) {
							// Large gap detected - possible packet loss
							int counter_diff = (int)counter - (int)last_ping_counter[tracker_id];
							if (counter_diff < 0) {
								counter_diff += 256;
							}
							if (counter_diff > 10) {
								LOG_WRN(
									"Large PING counter gap: id=%u last=%u new=%u (gap=%d)",
									tracker_id,
									last_ping_counter[tracker_id],
									counter,
									counter_diff
								);
							}
						}

						// Update last seen counter (only if not out-of-order and not skipped)
						if (!is_out_of_order) {
							last_ping_counter[tracker_id] = counter;
						}
					} // End of else branch for ping_counter_initialized

					if (ping_ack_flag != ESB_PONG_FLAG_NORMAL) {
						if (tracker_remote_command[tracker_id] == ping_ack_flag) {
							tracker_remote_command[tracker_id] = ESB_PONG_FLAG_NORMAL;
							LOG_DBG(
								"Tracker %u confirmed command 0x%02X, clearing "
								"flag",
								tracker_id,
								ping_ack_flag
							);

							if ((ping_ack_flag == ESB_PONG_FLAG_SET_CHANNEL
								 || ping_ack_flag == ESB_PONG_FLAG_CLEAR_CHANNEL)
								&& channel_change_pending) {
								atomic_or(&channel_ack_mask, (1 << tracker_id));
								uint8_t current_mask = atomic_get(&channel_ack_mask);
								LOG_INF(
									"Tracker %u confirmed channel change "
									"(%u/%u confirmed)",
									tracker_id,
									__builtin_popcount(current_mask),
									stored_trackers
								);
							}
						}
					}

					struct esb_payload pong = {.noack = false, .pipe = 1 + (tracker_id % 7), .length = ESB_PONG_LEN};

					pong.data[0] = ESB_PONG_TYPE;
					pong.data[1] = tracker_id;
					pong.data[2] = counter;
					pong.data[3] = (current_rx_ticks >> 24) & 0xFF;
					pong.data[4] = (current_rx_ticks >> 16) & 0xFF;
					pong.data[5] = (current_rx_ticks >> 8) & 0xFF;
					pong.data[6] = (current_rx_ticks) & 0xFF;
					pong.data[7] = tracker_remote_command[tracker_id];

					// Fill data[8-11] based on command type
					if (tracker_remote_command[tracker_id] == ESB_PONG_FLAG_SET_CHANNEL) {
						// For SET_CHANNEL, use the channel value
						pong.data[8] = (tracker_channel_value >> 24) & 0xFF;
						pong.data[9] = (tracker_channel_value >> 16) & 0xFF;
						pong.data[10] = (tracker_channel_value >> 8) & 0xFF;
						pong.data[11] = (tracker_channel_value) & 0xFF;
					} else if (tracker_remote_command[tracker_id] == ESB_PONG_FLAG_SENS_SET) {
						// For SENS_SET, use compressed sensitivity data
						// Overwrite time sync bytes (3-6) and use bytes 8-9
						pong.data[3] = (pending_sens_data[tracker_id][0] >> 8) & 0xFF;
						pong.data[4] = (pending_sens_data[tracker_id][0]) & 0xFF;
						pong.data[5] = (pending_sens_data[tracker_id][1] >> 8) & 0xFF;
						pong.data[6] = (pending_sens_data[tracker_id][1]) & 0xFF;

						pong.data[8] = (pending_sens_data[tracker_id][2] >> 8) & 0xFF;
						pong.data[9] = (pending_sens_data[tracker_id][2]) & 0xFF;
						pong.data[10] = 0; // Unused
						pong.data[11] = 0; // Unused
					} else if (tracker_remote_command[tracker_id] == ESB_PONG_FLAG_NORMAL) {
						// Send ticks_diff for clock skew compensation
						pong.data[8] = (ticks_diff >> 24) & 0xFF;
						pong.data[9] = (ticks_diff >> 16) & 0xFF;
						pong.data[10] = (ticks_diff >> 8) & 0xFF;
						pong.data[11] = (ticks_diff) & 0xFF;
					} else {
						// For other commands, currently unused
						memset(&pong.data[8], 0x00, 4); // reserved
					}

					// Try to write ACK payload with robust error handling
					pong.data[ESB_PONG_LEN - 1] = crc8_ccitt(0x07, pong.data, ESB_PONG_LEN - 1);
					esb_flush_tx();
					int werr = esb_write_payload(&pong);

					if (werr == 0) {
						// Success - mark this counter as queued
						last_pong_queued_counter[tracker_id] = counter;
						LOG_DBG(
							"ACK payload queued: PONG id=%u ctr=%u pipe=%u "
							"cmd=0x%02X",
							tracker_id,
							counter,
							pong.pipe,
							tracker_remote_command[tracker_id]
						);
					}
				}
			} break;
			case 17: // 16 bytes data + 1 byte sequence number
			{
				uint8_t tracker_id = rx_payload.data[1];

				// TDMA Slot Check for Data (Type 17)
				if (tracker_id < MAX_TRACKERS) {
					uint64_t slot_offset = current_rx_us % TDMA_PACKET_INTERVAL_US;
					uint64_t expected_start = tracker_id * TDMA_SLOT_DURATION_US;
					uint64_t expected_end = (tracker_id + 1) * TDMA_SLOT_DURATION_US;

					// Calculate signed offset from the center of the expected slot
					int64_t center_offset
						= (int64_t)slot_offset - (int64_t)(expected_start + TDMA_SLOT_DURATION_US / 2);

					// Update stats
					struct tdma_stats *stats = &g_tdma_stats[tracker_id];
					stats->count++;
					stats->sum_offset += center_offset;
					stats->sum_sq_offset += (center_offset * center_offset);
					if (stats->count == 1) {
						stats->min_offset = center_offset;
						stats->max_offset = center_offset;
					} else {
						if (center_offset < stats->min_offset) {
							stats->min_offset = center_offset;
						}
						if (center_offset > stats->max_offset) {
							stats->max_offset = center_offset;
						}
					}

					if (slot_offset < expected_start || slot_offset >= expected_end) {
						stats->violations++;
					}
				}

				if (tracker_id >= stored_trackers) { // not a stored tracker
					continue;
				}

				if (rx_payload.data[0] > 223) { // reserved for receiver only
					break;
				}

				uint8_t received_sequence = rx_payload.data[16];
				int seq_result = check_packet_sequence(tracker_id, received_sequence);
				// Decide whether to forward the packet based on the sequence check result
				// seq_result: 0=normal, 1=potential loss, 2=out of order, 3=reboot, 4=duplicate
				if (seq_result == 4) {
					LOG_DBG("TRK %d: Duplicate packet seq=%d, dropped", tracker_id, received_sequence);
					// Drop duplicate packet
					break;
				}
				if (seq_result == 2) {
					LOG_DBG("TRK %d: Out-of-order packet seq=%d, dropped", tracker_id, received_sequence);
					// Drop out-of-order packet to avoid incorrect pose calculation
					break;
				}

				// Forward packet for other cases (normal, potential loss, reboot)
				hid_write_packet_n(rx_payload.data,
								   rx_payload.rssi); // write to hid endpoint
			} break;
			case 16: // legacy format without sequence number, no TDMA check, just for backward compatibility
			{
				uint8_t imu_id = rx_payload.data[1];

				if (imu_id >= stored_trackers) { // not a stored tracker
					continue;
				}

				if (rx_payload.data[0] > 223) { // reserved for receiver only
					break;
				}
				hid_write_packet_n(rx_payload.data,
								   rx_payload.rssi); // write to hid endpoint
			} break;
			default:
				LOG_ERR("Wrong packet length: %d", rx_payload.length);
				break;
			}
		}
	} break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int fetch_attempts = 0;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (!clk_mgr) {
		LOG_ERR("Unable to get the Clock manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("Clock request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("Clock could not be started: %d", res);
			return res;
		}
		if (err && ++fetch_attempts > 10000) {
			LOG_WRN("Unable to fetch Clock request result: %d", err);
			return err;
		}
	} while (err);

	LOG_DBG("HF clock started");
	return 0;
}

// this was randomly generated
// TODO: I have no idea?
static const uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
static const uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
static const uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};

static uint8_t base_addr_0[4], base_addr_1[4], addr_prefix[8] = {0};

static bool esb_initialized = false;

int esb_initialize(bool tx)
{
	if (esb_initialized) {
		LOG_WRN("ESB already initialized");
	}
	int err;

	struct esb_config config = ESB_DEFAULT_CONFIG;

	if (tx) {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		// config.mode = ESB_MODE_PTX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		// config.retransmit_count = 0;
		config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
	} else {
		config.protocol = ESB_PROTOCOL_ESB_DPL;
		config.mode = ESB_MODE_PRX;
		config.event_handler = event_handler;
		// config.bitrate = ESB_BITRATE_2MBPS;
		// config.crc = ESB_CRC_16BIT;
		config.tx_output_power = CONFIG_RADIO_TX_POWER;
		config.retransmit_delay = RADIO_RETRANSMIT_DELAY;
		// config.retransmit_count = 3;
		// config.tx_mode = ESB_TXMODE_MANUAL;
		// config.payload_length = 32;
		config.selective_auto_ack = true;
		// config.use_fast_ramp_up = true;
	}

	LOG_INF("Initializing ESB, %sX mode", tx ? "T" : "R");
	err = esb_init(&config);

	if (!err) {
		// Use saved channel if available, otherwise use default
		uint8_t channel_to_use = (receiver_rf_channel <= 100) ? receiver_rf_channel : RADIO_RF_CHANNEL;
		esb_set_rf_channel(channel_to_use);
		LOG_INF("Set RF channel to %u", channel_to_use);
	}

	if (!err) {
		esb_set_base_address_0(base_addr_0);
	}

	if (!err) {
		esb_set_base_address_1(base_addr_1);
	}

	if (!err) {
		esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));
	}

	if (err) {
		LOG_ERR("ESB initialization failed: %d", err);
		set_status(SYS_STATUS_CONNECTION_ERROR, true);
		return err;
	}

	esb_initialized = true;
	return 0;
}

static void esb_deinitialize(void)
{
	LOG_INF("ESB deinitialize requested");
	if (esb_initialized) {
		esb_initialized = false;
		LOG_INF("Deinitializing ESB");
		k_msleep(10); // wait for pending transmissions
		if (esb_initialized) {
			LOG_INF("ESB denitialize cancelled");
			return;
		}
		esb_disable();
	}
	esb_initialized = false;
}

inline void esb_set_addr_discovery(void)
{
	memcpy(base_addr_0, discovery_base_addr_0, sizeof(base_addr_0));
	memcpy(base_addr_1, discovery_base_addr_1, sizeof(base_addr_1));
	memcpy(addr_prefix, discovery_addr_prefix, sizeof(addr_prefix));
}

inline void esb_set_addr_paired(void)
{
	// Generate addresses from device address
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is
													   // not actually guaranteed, see datasheet)
	uint8_t buf[6] = {0};
	memcpy(buf, addr, 6);
	uint8_t addr_buffer[16] = {0};
	for (int i = 0; i < 4; i++) {
		addr_buffer[i] = buf[i];
		addr_buffer[i + 4] = buf[i] + buf[4];
	}
	for (int i = 0; i < 8; i++) {
		addr_buffer[i + 8] = buf[5] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (addr_buffer[i] == 0x00 || addr_buffer[i] == 0x55
			|| addr_buffer[i] == 0xAA) { // Avoid invalid addresses (see nrf datasheet)
			addr_buffer[i] += 8;
		}
	}
	memcpy(base_addr_0, addr_buffer, sizeof(base_addr_0));
	memcpy(base_addr_1, addr_buffer + 4, sizeof(base_addr_1));
	memcpy(addr_prefix, addr_buffer + 8, sizeof(addr_prefix));
}

static bool esb_pairing = false;
static bool esb_paired = false;
static bool esb_clearing = false;

int esb_add_pair(uint64_t addr, bool checksum)
{
	if (addr == 0) {
		return -EINVAL;
	}

	bool new_entry = false;
	int assigned_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	for (int i = 0; i < stored_trackers; i++) {
		if (stored_tracker_addr[i] == addr) {
			assigned_id = i;
			break;
		}
	}

	if (assigned_id < 0) {
		if (stored_trackers >= MAX_TRACKERS) {
			k_mutex_unlock(&tracker_store_lock);
			LOG_WRN("Tracker storage full, cannot add %012llX", addr);
			return -ENOSPC;
		}
		assigned_id = stored_trackers;
		stored_tracker_addr[assigned_id] = addr;
		stored_trackers++;
		new_entry = true;
	}

	k_mutex_unlock(&tracker_store_lock);

	if (new_entry) {
		LOG_INF("Added device on id %d with address %012llX", assigned_id, addr);
		sys_write(STORED_ADDR_0 + assigned_id, NULL, &stored_tracker_addr[assigned_id], sizeof(stored_tracker_addr[0]));
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	} else {
		LOG_INF("Device already stored with id %d", assigned_id);
	}

	if (checksum) {
		uint8_t buf[6] = {0};
		memcpy(buf, &addr, 6);
		uint8_t checksum_byte = crc8_ccitt(0x07, buf, 6);
		if (checksum_byte == 0) {
			checksum_byte = 8;
		}
		// Use device address as unique identifier (although it is not actually guaranteed, see datasheet
		uint64_t *receiver_addr = (uint64_t *)NRF_FICR->DEVICEADDR;
		uint64_t pair_addr = (*receiver_addr & 0xFFFFFFFFFFFF) << 16;
		pair_addr |= checksum_byte;              // Add checksum to the address
		pair_addr |= (uint64_t)assigned_id << 8; // Add tracker id to the address
		LOG_INF("Pair the device with %016llX", pair_addr);
	}

	return assigned_id;
}

void esb_pop_pair(void)
{
	uint64_t removed_addr = 0;
	int removed_id = -1;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	if (stored_trackers > 0) {
		stored_trackers--;
		removed_id = stored_trackers;
		removed_addr = stored_tracker_addr[removed_id];
		stored_tracker_addr[removed_id] = 0;
	}
	k_mutex_unlock(&tracker_store_lock);

	if (removed_id >= 0) {
		sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
		sys_write(STORED_ADDR_0 + removed_id, NULL, &stored_tracker_addr[removed_id], sizeof(stored_tracker_addr[0]));
		LOG_INF("Removed device on id %d with address %012llX", removed_id, removed_addr);
	} else {
		LOG_WRN("No devices to remove");
	}
}

static bool esb_parse_pair(const uint8_t packet[8])
{
	uint64_t raw_addr = 0;
	memcpy(&raw_addr, packet, sizeof(raw_addr));
	uint64_t found_addr = (raw_addr >> 16) & 0xFFFFFFFFFFFF;
	uint8_t checksum = crc8_ccitt(0x07, &packet[2], 6);
	if (checksum == 0) {
		checksum = 8;
	}

	uint16_t send_tracker_id = 0;
	uint8_t tracker_count_snapshot = 0;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	tracker_count_snapshot = stored_trackers;
	send_tracker_id = tracker_count_snapshot; // default to next available ID
	for (uint8_t i = 0; i < tracker_count_snapshot; i++) {
		if (found_addr != 0 && stored_tracker_addr[i] == found_addr) {
			send_tracker_id = i;
			break;
		}
	}
	k_mutex_unlock(&tracker_store_lock);

	bool checksum_valid = (checksum == packet[0]);
	bool has_capacity = tracker_count_snapshot < MAX_TRACKERS;
	bool is_new_device = checksum_valid && found_addr != 0 && send_tracker_id == tracker_count_snapshot && has_capacity;
	bool ack_valid = false;

	if (is_new_device) {
		int assigned_id = esb_add_pair(found_addr, false);
		if (assigned_id >= 0) {
			send_tracker_id = (uint16_t)assigned_id;
			set_led(SYS_LED_PATTERN_ONESHOT_PROGRESS, SYS_LED_PRIORITY_HIGHEST);
		} else if (assigned_id == -ENOSPC) {
			LOG_WRN("Maximum tracker slots reached, cannot pair %012llX", found_addr);
		} else {
			LOG_ERR("Failed to store tracker address %012llX: %d", found_addr, assigned_id);
		}
	}

	ack_valid = checksum_valid && send_tracker_id < MAX_TRACKERS;

	tx_payload_pair.data[0] = ack_valid ? packet[0] : 0;
	tx_payload_pair.data[1] = (send_tracker_id < MAX_TRACKERS) ? (uint8_t)send_tracker_id : 0xFF;

	return ack_valid;
}

void esb_start_pairing(void)
{
	LOG_INF("Starting pairing mode (non-blocking)");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR;
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	k_msgq_purge(&esb_pairing_msgq);
}

void esb_pair(void)
{
	LOG_INF("Pairing");
	esb_set_addr_discovery();
	esb_initialize(false);
	esb_start_rx();
	tx_payload_pair.noack = false;
	uint64_t *addr = (uint64_t *)NRF_FICR->DEVICEADDR; // Use device address as unique identifier (although it is
													   // not actually guaranteed, see datasheet)
	memcpy(&tx_payload_pair.data[2], addr, 6);
	LOG_INF("Device address: %012llX", *addr & 0xFFFFFFFFFFFF);
	set_led(SYS_LED_PATTERN_SHORT, SYS_LED_PRIORITY_CONNECTION);
	esb_pairing = true;
	k_msgq_purge(&esb_pairing_msgq);
	while (esb_pairing) {
		if (!esb_initialized) {
			esb_initialize(false);
			esb_start_rx();
		}

		struct pairing_event evt;
		int q_err = k_msgq_get(&esb_pairing_msgq, &evt, K_MSEC(10));
		if (q_err != 0) {
			continue;
		}

		switch (evt.packet[1]) {
		case 0: {
			bool ack_ready = esb_parse_pair(evt.packet);
			if (!ack_ready) {
				LOG_DBG("Pairing request invalid, not queueing response");
				break;
			}
			int tx_err = esb_write_payload(&tx_payload_pair);
			if (tx_err == -ENOSPC) {
				esb_flush_tx();
				tx_err = esb_write_payload(&tx_payload_pair);
			}
			if (tx_err) {
				LOG_ERR("Failed to queue pairing response: %d", tx_err);
			} else {
				LOG_DBG("tx: %16llX", *(uint64_t *)tx_payload_pair.data);
			}
			break;
		}
		case 2:
			esb_flush_tx();
			break;
		case 1:
			// Tracker acknowledged previous response, nothing to do
			break;
		default:
			LOG_WRN("Unhandled pairing packet type %u", evt.packet[1]);
			break;
		}
	}
	set_led(SYS_LED_PATTERN_OFF, SYS_LED_PRIORITY_CONNECTION);
	esb_deinitialize();
}

void esb_reset_pair(void)
{
	esb_deinitialize(); // make sure esb is off
	esb_paired = false;
}

void esb_finish_pair(void)
{
	esb_pairing = false;
	k_msgq_purge(&esb_pairing_msgq);
}

void esb_clear(void)
{
	esb_clearing = true;

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t previous_count = stored_trackers;
	stored_trackers = 0;
	memset(stored_tracker_addr, 0, sizeof(stored_tracker_addr));
	k_mutex_unlock(&tracker_store_lock);

	sys_write(STORED_TRACKERS, NULL, &stored_trackers, sizeof(stored_trackers));
	for (uint8_t i = 0; i < previous_count && i < MAX_TRACKERS; i++) {
		sys_write(STORED_ADDR_0 + i, NULL, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	}
	LOG_INF("NVS Reset");

	esb_reset_pair();

	k_msleep(10);

	// Reset packet sequence state for all trackers
	for (int i = 0; i < MAX_TRACKERS; i++) {
		last_packet_sequence[i] = 0;
		packet_count[i] = 0;
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
	}
	LOG_INF("Packet sequence state and statistics reset for all trackers");

	hid_reset_all_rssi_smooth();
	esb_clearing = false;
}

// Reset packet sequence state for a specific tracker
void esb_reset_tracker_sequence(uint8_t tracker_id)
{
	if (tracker_id < MAX_TRACKERS) {
		last_packet_sequence[tracker_id] = 0;
		packet_count[tracker_id] = 0;
		// Reset PING counter tracking
		last_ping_counter[tracker_id] = 0;
		ping_counter_initialized[tracker_id] = false;
		last_pong_queued_counter[tracker_id] = 0;
		// Reset statistics
		memset(&tracker_stats[tracker_id], 0, sizeof(struct packet_stats));
		// Reset RSSI smoothing state
		hid_reset_rssi_smooth(tracker_id);
		LOG_INF("Packet sequence state and statistics reset for tracker %d", tracker_id);
	}
}

void esb_send_remote_command_sens(uint8_t tracker_id, float x, float y, float z)
{
	if (tracker_id >= MAX_TRACKERS) {
		return;
	}
	pending_sens_data[tracker_id][0] = (int16_t)(x * 100.0f);
	pending_sens_data[tracker_id][1] = (int16_t)(y * 100.0f);
	pending_sens_data[tracker_id][2] = (int16_t)(z * 100.0f);
	tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SENS_SET;
	LOG_INF("Queued SENS_SET for tracker %u: %.2f, %.2f, %.2f", tracker_id, (double)x, (double)y, (double)z);
}

void esb_send_remote_command_channel(uint8_t tracker_id, uint8_t channel)
{
	if (tracker_id < MAX_TRACKERS) {
		tracker_remote_command[tracker_id] = ESB_PONG_FLAG_SET_CHANNEL;
		tracker_channel_value = channel;
		LOG_INF("Queued SET_CHANNEL %u for tracker %u", channel, tracker_id);
	}
}

// Send remote command to specified tracker
void esb_send_remote_command(uint8_t tracker_id, uint8_t command_flag)
{
	if (tracker_id < MAX_TRACKERS) {
		tracker_remote_command[tracker_id] = command_flag;
		const char *cmd_name = "UNKNOWN";
		switch (command_flag) {
		case ESB_PONG_FLAG_NORMAL:
			cmd_name = "NORMAL";
			break;
		case ESB_PONG_FLAG_SHUTDOWN:
			cmd_name = "SHUTDOWN";
			break;
		case ESB_PONG_FLAG_CALIBRATE:
			cmd_name = "CALIBRATE";
			break;
		case ESB_PONG_FLAG_SIX_SIDE_CAL:
			cmd_name = "SIX_SIDE_CAL";
			break;
		case ESB_PONG_FLAG_MEOW:
			cmd_name = "MEOW";
			break;
		case ESB_PONG_FLAG_SCAN:
			cmd_name = "SCAN";
			break;
		case ESB_PONG_FLAG_MAG_CLEAR:
			cmd_name = "MAG_CLEAR";
			break;
		case ESB_PONG_FLAG_REBOOT:
			cmd_name = "REBOOT";
			break;
		case ESB_PONG_FLAG_CLEAR:
			cmd_name = "CLEAR";
			break;
		case ESB_PONG_FLAG_DFU:
			cmd_name = "DFU";
			break;
		case ESB_PONG_FLAG_SET_CHANNEL:
			cmd_name = "SET_CHANNEL";
			break;
		case ESB_PONG_FLAG_SENS_SET:
			cmd_name = "SENS_SET";
			break;
		case ESB_PONG_FLAG_SENS_RESET:
			cmd_name = "SENS_RESET";
			break;
		case ESB_PONG_FLAG_RESET_ZRO:
			cmd_name = "RESET_ZRO";
			break;
		case ESB_PONG_FLAG_RESET_ACC:
			cmd_name = "RESET_ACC";
			break;
		case ESB_PONG_FLAG_RESET_BAT:
			cmd_name = "RESET_BAT";
			break;
		case ESB_PONG_FLAG_PING:
			cmd_name = "PING";
			break;
		case ESB_PONG_FLAG_RESET_TCAL:
			cmd_name = "RESET_TCAL";
			break;
		case ESB_PONG_FLAG_TCAL_AUTO_ON:
			cmd_name = "TCAL_AUTO_ON";
			break;
		case ESB_PONG_FLAG_TCAL_AUTO_OFF:
			cmd_name = "TCAL_AUTO_OFF";
			break;
		case ESB_PONG_FLAG_FUSION_RESET:
			cmd_name = "FUSION_RESET";
			break;
		case ESB_PONG_FLAG_TCAL_BOOT_ON:
			cmd_name = "TCAL_BOOT_ON";
			break;
		case ESB_PONG_FLAG_TCAL_BOOT_OFF:
			cmd_name = "TCAL_BOOT_OFF";
			break;
		}
		LOG_INF("Remote command %s (0x%02X) queued for tracker %d", cmd_name, command_flag, tracker_id);
	} else {
		LOG_ERR("Invalid tracker ID: %d", tracker_id);
	}
}

// Send remote command to all paired trackers
void esb_send_remote_command_all(uint8_t command_flag)
{
	uint8_t count = 0;
	const char *cmd_name = "UNKNOWN";
	switch (command_flag) {
	case ESB_PONG_FLAG_NORMAL:
		cmd_name = "NORMAL";
		break;
	case ESB_PONG_FLAG_SHUTDOWN:
		cmd_name = "SHUTDOWN";
		break;
	case ESB_PONG_FLAG_CALIBRATE:
		cmd_name = "CALIBRATE";
		break;
	case ESB_PONG_FLAG_SIX_SIDE_CAL:
		cmd_name = "SIX_SIDE_CAL";
		break;
	case ESB_PONG_FLAG_MEOW:
		cmd_name = "MEOW";
		break;
	case ESB_PONG_FLAG_SCAN:
		cmd_name = "SCAN";
		break;
	case ESB_PONG_FLAG_MAG_CLEAR:
		cmd_name = "MAG_CLEAR";
		break;
	case ESB_PONG_FLAG_REBOOT:
		cmd_name = "REBOOT";
		break;
	case ESB_PONG_FLAG_CLEAR:
		cmd_name = "CLEAR";
		break;
	case ESB_PONG_FLAG_DFU:
		cmd_name = "DFU";
		break;
	case ESB_PONG_FLAG_SET_CHANNEL:
		cmd_name = "SET_CHANNEL";
		break;
	case ESB_PONG_FLAG_SENS_SET:
		cmd_name = "SENS_SET";
		break;
	case ESB_PONG_FLAG_SENS_RESET:
		cmd_name = "SENS_RESET";
		break;
	case ESB_PONG_FLAG_RESET_ZRO:
		cmd_name = "RESET_ZRO";
		break;
	case ESB_PONG_FLAG_RESET_ACC:
		cmd_name = "RESET_ACC";
		break;
	case ESB_PONG_FLAG_RESET_BAT:
		cmd_name = "RESET_BAT";
		break;
	case ESB_PONG_FLAG_PING:
		cmd_name = "PING";
		break;
	case ESB_PONG_FLAG_RESET_TCAL:
		cmd_name = "RESET_TCAL";
		break;
	case ESB_PONG_FLAG_TCAL_AUTO_ON:
		cmd_name = "TCAL_AUTO_ON";
		break;
	case ESB_PONG_FLAG_TCAL_AUTO_OFF:
		cmd_name = "TCAL_AUTO_OFF";
		break;
	case ESB_PONG_FLAG_FUSION_RESET:
		cmd_name = "FUSION_RESET";
		break;
	case ESB_PONG_FLAG_TCAL_BOOT_ON:
		cmd_name = "TCAL_BOOT_ON";
		break;
	case ESB_PONG_FLAG_TCAL_BOOT_OFF:
		cmd_name = "TCAL_BOOT_OFF";
		break;
	}

	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	for (uint8_t i = 0; i < stored_trackers && i < MAX_TRACKERS; i++) {
		if (stored_tracker_addr[i] != 0) {
			tracker_remote_command[i] = command_flag;
			count++;
		}
	}
	k_mutex_unlock(&tracker_store_lock);

	LOG_INF("Remote command %s (0x%02X) queued for %d trackers", cmd_name, command_flag, count);
}

// Manually print statistics for all active trackers
void esb_print_all_stats(void)
{
	LOG_INF("=== Packet Statistics Summary ===");
	print_tracker_stats_batch();
	LOG_INF("================================");
}

void esb_reset_all_stats(void)
{
	for (int i = 0; i < MAX_TRACKERS; i++) {
		memset(&tracker_stats[i], 0, sizeof(struct packet_stats));
		ping_counter_initialized[i] = false;
		last_ping_counter[i] = 0;
		last_pong_queued_counter[i] = 0;
	}
	memset(&tx_statistics, 0, sizeof(struct tx_stats));
	LOG_INF("All packet statistics, ACK statistics, and TX statistics have been reset");
}

// Sets the RF channel for all trackers
void esb_set_all_trackers_channel(uint8_t channel)
{
	if (channel > 100) {
		LOG_ERR("Invalid channel value: %u (must be 0-100)", channel);
		return;
	}

	if (channel_change_pending) {
		LOG_WRN("Channel change already in progress, please wait");
		return;
	}

	tracker_channel_value = channel;
	pending_channel = channel;
	channel_change_pending = true;
	atomic_set(&channel_ack_mask, 0);
	channel_change_timeout = k_uptime_get() + CHANNEL_CHANGE_TIMEOUT_MS;

	esb_send_remote_command_all(ESB_PONG_FLAG_SET_CHANNEL);
	LOG_INF("RF channel %u command sent to all trackers, waiting for confirmation...", channel);
}

// Clear RF channel settings for all trackers (restore to default)
void esb_clear_all_trackers_channel(void)
{
	if (channel_change_pending) {
		LOG_WRN("Channel change already in progress, please wait");
		return;
	}

	pending_channel = 0xFF; // Special value to indicate clearing
	channel_change_pending = true;
	atomic_set(&channel_ack_mask, 0);
	channel_change_timeout = k_uptime_get() + CHANNEL_CHANGE_TIMEOUT_MS;

	esb_send_remote_command_all(ESB_PONG_FLAG_CLEAR_CHANNEL);
	LOG_INF("CLEAR_CHANNEL command sent to all trackers, waiting for confirmation...");
}

// Set the receiver's RF channel (local, does not affect trackers)
void esb_set_receiver_channel(uint8_t channel)
{
	if (channel > 100) {
		LOG_ERR("Invalid channel value: %u (must be 0-100)", channel);
		return;
	}

	LOG_INF("Setting receiver RF channel to %u (local only)", channel);
	receiver_rf_channel = channel;

	// Save to NVS
	sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

	// Reinitialize ESB with new channel
	esb_deinitialize();
	esb_initialize(false);
	esb_start_rx();

	LOG_INF("Receiver channel switched to %u", receiver_rf_channel);
}

// Clear the receiver's RF channel setting (local, does not affect trackers)
void esb_clear_receiver_channel(void)
{
	LOG_INF("Clearing receiver RF channel (local only)");
	receiver_rf_channel = 0xFF;

	// Clear from NVS
	sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

	// Reinitialize ESB with default channel
	esb_deinitialize();
	esb_initialize(false);
	esb_start_rx();

	LOG_INF("Receiver channel cleared, using default");
}

// Get the receiver's RF channel
uint8_t esb_get_receiver_channel(void)
{
	return receiver_rf_channel;
}

// TODO:
void esb_write_sync(uint16_t led_clock)
{
	if (!esb_initialized || !esb_paired) {
		return;
	}
	tx_payload_sync.noack = false;
	tx_payload_sync.data[0] = (led_clock >> 8) & 255;
	tx_payload_sync.data[1] = led_clock & 255;
	esb_write_payload(&tx_payload_sync);
}

// TODO:
void esb_receive(void)
{
	esb_set_addr_paired();
	esb_paired = true;
}

static void esb_thread(void)
{
	clocks_start();

	sys_read(STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
	k_mutex_lock(&tracker_store_lock, K_FOREVER);
	uint8_t tracker_count = stored_trackers;
	for (uint8_t i = 0; i < tracker_count && i < MAX_TRACKERS; i++) {
		sys_read(STORED_ADDR_0 + i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
	}
	k_mutex_unlock(&tracker_store_lock);

	// Load saved RF channel from NVS if exists
	uint8_t saved_channel = 0xFF;
	sys_read(RF_CHANNEL, &saved_channel, sizeof(saved_channel));
	// 0xFF and 0 both indicate "use default"
	if (saved_channel != 0xFF && saved_channel != 0 && saved_channel <= 100) {
		// Valid saved channel found, use it
		receiver_rf_channel = saved_channel;
		LOG_INF("Loaded RF channel %u from NVS", saved_channel);
	} else {
		LOG_INF("No saved RF channel, using default %u", RADIO_RF_CHANNEL);
		// If channel was 0 (uninitialized), write 0xFF to NVS
		if (saved_channel == 0) {
			saved_channel = 0xFF;
			sys_write(RF_CHANNEL, NULL, &saved_channel, sizeof(saved_channel));
		}
	}

	if (tracker_count) {
		esb_paired = true;
	}
	LOG_INF("%d/%d devices stored", tracker_count, MAX_TRACKERS);

	if (esb_paired) {
		esb_receive();
		esb_initialize(false);
		esb_start_rx();
	}

		while (1) {
		if (!esb_paired && !esb_clearing) {
			esb_pair();
			esb_receive();
			esb_initialize(false);
			esb_start_rx();
		}

		// Periodic antenna switching for diversity
		antenna_periodic_switch();

		// Check if channel change is complete
		if (channel_change_pending) {
			uint8_t expected_mask = (1 << stored_trackers) - 1;
			uint8_t current_mask = atomic_get(&channel_ack_mask);
			int64_t now = k_uptime_get();

			if (current_mask == expected_mask) {
				// All trackers confirmed, switch receiver channel
				if (pending_channel == 0xFF) {
					// Clear channel setting
					LOG_INF("All trackers confirmed channel clear, restoring receiver to default");
					receiver_rf_channel = 0xFF;

					// Clear from NVS
					sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

					// Reinitialize ESB with default channel
					esb_deinitialize();
					esb_initialize(false);
					esb_start_rx();

					LOG_INF("Receiver channel cleared, using default %u", RADIO_RF_CHANNEL);
				} else {
					// Set new channel
					LOG_INF("All trackers confirmed channel change to %u, switching receiver", pending_channel);
					receiver_rf_channel = pending_channel;

					// Save to NVS
					sys_write(RF_CHANNEL, NULL, &receiver_rf_channel, sizeof(receiver_rf_channel));

					// Reinitialize ESB with new channel
					esb_deinitialize();
					esb_initialize(false);
					esb_start_rx();

					LOG_INF("Receiver channel switched to %u successfully", receiver_rf_channel);
				}

				channel_change_pending = false;
			} else if (now >= channel_change_timeout) {
				// Timeout, cancel channel change
				LOG_WRN(
					"Channel change timeout, %u/%u trackers confirmed",
					__builtin_popcount(current_mask),
					stored_trackers
				);
				channel_change_pending = false;
				tracker_channel_value = 0; // Reset pending channel value
			}
		}

		k_msleep(100);
	}
}
