# Changelog

All notable changes to this project will be documented in this file. The format
is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and the
project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
