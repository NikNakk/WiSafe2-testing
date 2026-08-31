# Changelog

All notable changes to this project will be documented in this file. The format
is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the
project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Per-detector Home Assistant device triggers for alarm detected, alarm cleared,
  physical test passed and physical test failed events.
- A per-detector `Network member` diagnostic entity derived from the latest SID
  map, so stale retained inventory is visible without automatically deleting
  Home Assistant devices after a radio reset or temporary topology change.
- The ST-630-DE smoke detector model mapping (`7C04`) documented by ws2mqtt.

### Changed

- MQTT sends now use ESPHome's asynchronous ESP-IDF path, and radio events are
  processed in bounded main-loop batches. This prevents detector discovery and
  diagnostic publication from blocking other ESPHome components for seconds.

### Documentation

- Clarified why remote diagnostic battery bytes and flags cannot safely be used
  to infer the detector's low-battery or on-base state, and why MQTT detector
  devices do not declare an ESPHome-native bridge as `via_device`.

## [0.5.0] - 2026-08-31

### Added

- Active detector discovery from the radio SID map, including `C4` remote
  identity decoding and per-detector `D4 06` diagnostic polling.
- Immediate discovery of newly paired detectors announced with `D4 09`, plus
  raw battery, RSSI, firmware, diagnostic-flag, and radio-fault entities.
- An on-demand `Refresh Detector Diagnostics` button that queries every known
  detector without enabling periodic per-detector polling.

### Fixed

- Treat `46 7E` as acceptance of remote identity requests, wait for their `C4`
  identity frames asynchronously, and enforce a 500 ms quiet interval between
  radio frames before issuing another command. This prevents SID discovery and
  remote diagnostic requests from being sent faster than the donor radio can
  process them.
- Serialize remote identity and diagnostic work so only one request awaits its
  asynchronous response at a time, and defer local polling while it is
  outstanding. This prevents delayed `D4 06` responses colliding with later
  commands and losing their leading `D4` byte.
- Ignore `D4 09` announcements when the SID and device ID already match the
  stored inventory instead of unnecessarily repeating identity discovery.
- Reconcile the SID map before a manual detector refresh, avoiding a misleading
  `NO DETECTORS` result merely because discovery had not yet run after startup.
- Apply WiSafe2 reserved-byte stuffing on every SPI transmit and receive path:
  payload `7E` and `7D` bytes are now encoded as `7D 01` and `7D 02`, while the
  final `7E` frame delimiter remains unescaped.
- Decode `51` alarm-off events so remote alarm entities clear when the radio
  reports that an interlinked alarm has ended.
- Only explicit alarm-on and alarm-off events now change alarm state; routine
  test, status, and diagnostic traffic no longer clears an active alarm.

## [0.4.0] - 2026-08-29

### Added

- Customizable Home Assistant blueprints for notifications from real smoke,
  heat, and carbon-monoxide alarm events and from physical detector tests.
- A conservative 60-second local radio diagnostic/SID-map poll and automatic
  identity replies when the radio requests them with `41 7E`.

### Fixed

- Decode `D2` frames as attached-radio diagnostics instead of incorrectly
  creating `MISSING` detector events for the bridge identity.

## [0.3.0] - 2026-08-29

### Added

- Native ESPHome controls for fire, CO, and combined interlink sound tests;
  fire and CO silence commands; pairing-state queries; and pairing initiation.
- Pairing-state, command-running, and last-command diagnostic entities.
- Host tests for every outbound management-command frame.
- Coverage for extended received frames with trailing, currently unknown data.
- A suggested auto-entities/Mushroom Home Assistant alarm dashboard card.
- Persistent per-detector last-test results and timestamp entities.

### Changed

- Shortened detector entity names to `Battery` and `Base`.
- Aligned known detector model/type inference with the existing Home Assistant
  integration, including the WST-630N (`340E`) smoke alarm.
- Consolidated radio exchanges onto one scratch buffer and made command-result
  event delivery non-blocking.

## [0.2.0] - 2026-08-28

### Added

- Decoding for WiSafe2 status, test, emergency, silence, and missing-detector
  packets.
- Persistent detector inventory with dynamic Home Assistant MQTT discovery and
  one Home Assistant device per detected alarm.
- Native API diagnostic entities for the bridge and decoded packet state.
- Host-side protocol tests and GitHub Actions validation of both the decoder and
  complete ESPHome firmware build.

### Changed

- Made receive framing recover from incomplete and overflowing packets.
- Republish detector discovery and retained state after MQTT reconnects.

## [0.1.0] - 2026-08-28

### Added

- Initial ESPHome external-component prototype for radio initialization and raw
  packet reception on an M5Stack AtomS3 Lite.
