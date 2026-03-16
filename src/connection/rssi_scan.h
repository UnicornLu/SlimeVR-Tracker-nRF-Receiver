/*
 * SlimeVR Code is placed under the MIT license
 * Copyright (c) 2026 SlimeVR Contributors
 */
#pragma once

/**
 * Run an RSSI scan across ESB channels 1..100 and print per-channel results +
 * a recommended channel.
 *
 * Notes:
 * - This is a blocking call intended for manual console use.
 * - The scan will temporarily stop ESB RX and reinitialize ESB RX when done.
 */
void rssi_scan_run_and_print(void);
