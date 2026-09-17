# SlimeNRF receiver firmware

Zephyr/NCS firmware for SlimeNRF receivers on Nordic nRF52833 and nRF52840 SoCs.

## Project history

This firmware is originally based on [SlimeVR/SlimeVR-Tracker-nRF-Receiver](https://github.com/SlimeVR/SlimeVR-Tracker-nRF-Receiver). Thanks to the upstream contributors for their work.

This fork continues development alongside our [tracker firmware](https://github.com/jitingcn/SlimeVR-Tracker-nRF), with changes driven by real players and community needs.

## Major differences from upstream

- Remote tracker commands through the receiver console and USB HID
- Different TDMA radio scheduling, adaptive rate control, and clock synchronization
- ESB OTA updates for trackers and receiver self-update support
- Single-tracker and batch raw sensor collection over USB HID or CDC
- On-demand repair of missing collection metadata and calibration tables
- Python tools for data collection, remote commands, and OTA updates
- Optional per-tracker radio and clock-sync diagnostics

Use matching tracker, receiver, and host-tool versions for fork-specific features.

## SDK and build environment

`west.yml` selects [jitingcn/sdk-nrf](https://github.com/jitingcn/sdk-nrf)
`v3.4-branch`, based on the official NCS `v3.4.0` release.

Builds require Zephyr SDK **1.0.1 GNU** (`zephyr/gnu`, GCC 14.3.0) and
**Python 3.12**. The firmware uses Picolibc; CI runs on Ubuntu 24.04.

The firmware depends on the SDK's ESB extensions and USB fixes; stock NCS
is not a drop-in replacement. UF2 generation uses `CONFIG_BUILD_OUTPUT_HEX=y`
to read image addresses from HEX output. The SDK includes the
[upstream HEX-first UF2 fix](https://github.com/zephyrproject-rtos/zephyr/pull/107944)
required for mapped partitions.

## Raw collection metadata

Update the tracker firmware, receiver firmware, and collector together for
on-demand metadata repair. New collection sessions reserve their first five
seconds for metadata/calibration packets before transmitting raw samples.
Metadata is no longer resent every minute. Calibration repairs use the same
snapshot captured at session start; restart collection to capture new calibration.

During single or batch collection, the receiver holds the published loss-ladder
level and cancels unpublished ladder changes. This controller measures the normal
fusion stream, not raw capture loss. Genuine tracker joins/leaves still update
membership and may change the layout; normal loss adaptation resumes after capture.

Batch raw stream packets use NoACK without hardware retries, so a retry cannot
overrun the slot reserved for one sample. Metadata, calibration, PING, and reliable
single-target collection retain their ACK behavior. Batch loss remains possible
at admission, radio, or USB stages; the collector's sequence-gap percentage does
not identify the failing stage by itself.

The web analyzer needs its receiver **control** connection as well as its data
connection to request missing metadata. The Python HID/CDC collectors can discover
the command HID interface with the same receiver serial number. If discovery is
unavailable, pass `--control-port <console-port>` to either collector. For CDC this
is the receiver console port, **not** the raw-data CDC port. Collectors warn when
no control connection is available; receiving data alone cannot repair losses.

Repair requests are rate-limited and stop once all sections and temperature-table
chunks have arrived. Meta files retain `tcal_declared_points` and `tcal_complete`;
incomplete temperature tables are not exported as valid `gyro_tcal` tables.

For manual repair, use the receiver console:

```text
collectmeta <tracker_id> <mask> <chunk>
```

The decimal mask combines: `1` basic metadata, `2` accelerometer calibration,
`4` magnetometer calibration, `8` gyroscope calibration, `16` temperature-table
state, and `32` temperature-table points. Chunk indices are zero-based, with two
points per chunk; `255` requests all chunks. For example, `collectmeta 3 32 7`
requests only chunk 7 from tracker 3, and `collectmeta 3 63 255` requests everything.
Control HID opcode `223` carries the same three bytes: tracker ID, mask, chunk.

## License
Unless otherwise specified, all code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](LICENSE-MIT) or https://opensource.org/license/mit/)
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or https://opensource.org/license/apache-2-0/)

at your option. This means you can select the license you prefer!

Unless you explicitly state otherwise, any contribution intentionally submitted for
inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual
licensed as above, without any additional terms or conditions.
