#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MAX_TRACKERS 8
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ARG_UNUSED(x) (void)(x)
#define LOG_INF(...)
#define LOG_WRN(...)
#define LOG_DBG(...)
#define TPS_CALCULATION_INTERVAL_MS 1000

static int64_t now = 1000;
static uint32_t tdma_active_mask = BIT(3);
static int test_all_state_valid;
static int test_all_enabled;
static bool collecting;
static bool batching;
static bool ota;
static uint8_t last_packet_sequence[MAX_TRACKERS];
static uint32_t packet_count[MAX_TRACKERS];
static uint8_t last_ping_counter[MAX_TRACKERS];
static uint8_t last_pong_queued_counter[MAX_TRACKERS];

static int64_t k_uptime_get(void)
{
	return now;
}

static int atomic_get(const int *value)
{
	return *value;
}

static bool data_collect_is_active(void)
{
	return collecting;
}

static bool data_collect_batch_is_active(void)
{
	return batching;
}

static bool esb_ota_relay_is_active(void)
{
	return ota;
}

static void test_all_invalidate_tracker(uint8_t id, int64_t at)
{
	ARG_UNUSED(id);
	ARG_UNUSED(at);
}

/* PRODUCTION BODIES */

static void tick(uint32_t received, uint32_t gaps)
{
	tracker_stats[3].total_received += received;
	tracker_stats[3].total_gaps += gaps;
	now += 1000;
	tdma_loss_controller_tick(now);
}

static void repeat(unsigned ticks, uint32_t received, uint32_t gaps)
{
	for (unsigned i = 0; i < ticks; i++) {
		tick(received, gaps);
	}
}

static void fast_recovery(void)
{
	repeat(29, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void slow_probe(void)
{
	repeat(119, 196, 4);
	assert(tdma_cap_level == 3);
	tick(196, 4);
	assert(tdma_cap_level == 2);
}

static void sparse_recovery(void)
{
	/* 24-25 received+gap samples/tick, ~2% loss, qualifies every 5 ticks. */
	for (unsigned i = 0; i < 119; i++) {
		tick(24, i % 2);
	}
	assert(tdma_cap_level == 3);
	tick(24, 1);
	assert(tdma_cap_level == 2);
	tdma_published_cap_level = 2;

	/* 40 samples/tick, 2.5% loss, qualifies every 3 ticks. */
	repeat(119, 39, 1);
	assert(tdma_cap_level == 2);
	tick(39, 1);
	assert(tdma_cap_level == 1);
}

static void idle_breaks_evidence(void)
{
	repeat(119, 196, 4);
	repeat(180, 0, 0);
	assert(tdma_cap_level == 3);
	tick(196, 4);
	assert(tdma_cap_level == 3);
	repeat(119, 196, 4);
	assert(tdma_cap_level == 2);
}

static void loss_blocks_probe(void)
{
	repeat(119, 196, 4);
	tick(180, 20);
	repeat(119, 196, 4);
	assert(tdma_cap_level == 3);
	repeat(4, 180, 20);
	assert(tdma_cap_level == 3);
	tick(180, 20);
	assert(tdma_cap_level == 4);
}

static void threshold_boundaries(void)
{
	repeat(120, 190, 10); /* Exactly 5%: neither direction. */
	assert(tdma_cap_level == 3);
	repeat(30, 198, 2); /* Exactly 1%: no fast recovery. */
	assert(tdma_cap_level == 3);
	repeat(5, 1899, 101); /* 5.05% must not truncate to the neutral band. */
	assert(tdma_cap_level == 4);
}

static void counter_reset(void)
{
	repeat(29, 200, 0);
	tracker_stats[3].total_received = 0;
	tick(200, 0);
	assert(tdma_cap_level == 3);
	repeat(29, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void counter_wrap(void)
{
	tracker_stats[3].total_received = UINT32_MAX - 100;
	tdma_prev_recv[3] = tracker_stats[3].total_received;
	tick(200, 0); /* Counter wrap is a discarded discontinuity, not huge loss. */
	assert(tdma_cap_level == 3);
	repeat(29, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void restart_breaks_evidence(void)
{
	repeat(29, 200, 0);
	tracker_stats[3].restart_events++;
	tick(200, 0);
	assert(tdma_cap_level == 3);
	repeat(29, 200, 0);
	assert(tdma_cap_level == 3);
	tick(200, 0);
	assert(tdma_cap_level == 2);
}

static void ping_restart_sequence(void)
{
	assert(check_packet_sequence(3, 230) == 0);
	restart_ping(3);
	assert(check_packet_sequence(3, 0) == 0);
	assert(tracker_stats[3].total_gaps == 0);
	assert(tracker_stats[3].total_received == 2);
	assert(tdma_cap_level == 3); /* Reboot is not permission to raise rate. */
}

static void sequence_wrap(void)
{
	assert(check_packet_sequence(3, 254) == 0);
	assert(check_packet_sequence(3, 255) == 0);
	assert(check_packet_sequence(3, 0) == 0);
	assert(check_packet_sequence(3, 0) == 4);
	assert(check_packet_sequence(3, 255) == 2);
	assert(check_packet_sequence(3, 2) == 1);
	assert(tracker_stats[3].total_gaps == 1);
	assert(tracker_stats[3].restart_events == 0);
}

static void pending_publication(void)
{
	repeat(120, 196, 4);
	assert(tdma_cap_level == 2);
	repeat(180, 196, 4);
	assert(tdma_cap_level == 2);
	tdma_published_cap_level = 2;
	repeat(119, 196, 4);
	assert(tdma_cap_level == 2);
	tick(196, 4);
	assert(tdma_cap_level == 1);
}

static void excluded_modes(void)
{
	repeat(119, 196, 4);
	collecting = true;
	repeat(180, 196, 4);
	collecting = false;
	batching = true;
	repeat(180, 196, 4);
	batching = false;
	ota = true;
	repeat(180, 196, 4);
	ota = false;
	test_all_state_valid = 1;
	test_all_enabled = 1;
	repeat(180, 196, 4);
	test_all_enabled = 0;
	assert(tdma_cap_level == 3);
	repeat(119, 196, 4);
	assert(tdma_cap_level == 3);
	tick(196, 4);
	assert(tdma_cap_level == 2);
}

static const struct {
	const char *name;
	void (*run)(void);
} scenarios[] = {
	{"fast-recovery", fast_recovery},
	{"slow-probe", slow_probe},
	{"sparse-recovery", sparse_recovery},
	{"idle-breaks-evidence", idle_breaks_evidence},
	{"loss-blocks-probe", loss_blocks_probe},
	{"threshold-boundaries", threshold_boundaries},
	{"counter-reset", counter_reset},
	{"counter-wrap", counter_wrap},
	{"restart-breaks-evidence", restart_breaks_evidence},
	{"ping-restart-sequence", ping_restart_sequence},
	{"sequence-wrap", sequence_wrap},
	{"pending-publication", pending_publication},
	{"excluded-modes", excluded_modes},
};

int main(int argc, char **argv)
{
	assert(argc == 2);
	tdma_cap_level = 3;
	tdma_published_cap_level = 3;

	for (unsigned i = 0; i < ARRAY_SIZE(scenarios); i++) {
		if (strcmp(argv[1], scenarios[i].name) == 0) {
			scenarios[i].run();
			return 0;
		}
	}

	fprintf(stderr, "Unknown case: %s\n", argv[1]);
	return 2;
}
