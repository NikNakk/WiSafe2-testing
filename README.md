# WiSafe2 testing

Minimal ESP-IDF test harness for connecting a FireAngel WiSafe2 radio directly to an M5Stack AtomS3 Lite (ESP32-S3), without the Arduino Nano or 5 V/3.3 V level shifters used by the original bridge.

Based on the SPI/IRQ behaviour in [C19HOP/WiSafe2-to-HomeAssistant-Bridge](https://github.com/C19HOP/WiSafe2-to-HomeAssistant-Bridge).

## Important

This is experimental bench-test firmware. It is **not** part of the life-safety function of the FireAngel alarms and should not be relied on for alarm protection.

The WiSafe2 radio is the **SPI master** and the AtomS3 Lite is the **SPI slave**.

## Prototype wiring

The pin mapping below follows the radio connector nets traced from the original bridge PCB.

| WiSafe2 radio | AtomS3 Lite | Notes |
|---|---:|---|
| CS | GPIO8 | Radio -> Atom |
| IRQ | GPIO38 | Atom -> radio handshake |
| MOSI | GPIO6 | Radio -> Atom |
| SCK | GPIO5 | Radio -> Atom |
| MISO | GPIO7 | Atom -> radio |
| GND | GND | Common ground |
| 3V3 #1 | 3V3 | Supply |
| 3V3 #2 | 3V3 | Bridge to same 3.3 V supply |

The separate antenna connector has an ANT/RF pin and GND. For the prototype, connect the antenna as in the original bridge design and keep its ground common with the Atom/radio ground.

Do **not** connect the radio to 5 V logic. The AtomS3 Lite GPIO and the radio side of the original bridge are 3.3 V.

## What the test does

1. Configures the ESP32-S3 as an SPI slave.
2. Waits 5 seconds for the radio to stabilise, matching the original Nano firmware.
3. Sends the original init command byte-by-byte using the IRQ handshake:

   `D3 19 50 00 7E`

4. Waits for the expected response:

   `46 7E`

5. Queries the radio's pairing state with `D3 03 7E`.
6. If it is unpaired, reproduces the original bridge's pairing sequence:

   - sends `D3 12 01 7E`
   - requires `46 7E` followed by `41 7E`
   - announces the bridge identity as pseudo-device `A5 B8 13`, model `11 03`
   - opens a 21-second window in which you press the other FireAngel device's
     physical test/pair button
   - queries the pairing state again and reports the result

7. Enters raw receive mode and prints bytes/packets ending in `7E`.

The first goal is simply to establish whether ESP-IDF's SPI-slave driver can reproduce the timing expected by the WiSafe2 radio.

## Build and flash

Install a current ESP-IDF release, then from the repository root:

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

The serial port will vary by OS. On macOS it will normally appear under `/dev/cu.*`; run `ls /dev/cu.*` to identify it.

Exit the ESP-IDF monitor with `Ctrl+]`.

## ESPHome prototype

The repository also contains a local ESPHome external component in
`components/wisafe2`. It uses ESP-IDF's SPI-slave driver directly; do not add an
ESPHome `spi:` block because the WiSafe2 radio, rather than the AtomS3, is the
SPI master.

Copy `secrets.yaml.example` to `secrets.yaml`, enter the Wi-Fi and MQTT broker
credentials, and then compile or flash with:

```bash
esphome compile wisafe2.yaml
esphome run wisafe2.yaml
```

The ESPHome component publishes `Radio Initialized`, `Last Packet` and decoded
last-device diagnostics through the native API. A dedicated FreeRTOS task owns
the timing-sensitive SPI operations.

Each newly heard detector ID is stored in flash and advertised with retained
Home Assistant MQTT discovery messages. Home Assistant creates one device per
detector, containing alarm, low-battery, base-problem, model, last-event, test
result and raw-frame entities. Unused detector capacity is not advertised. The
default inventory limit is 16 and can be changed with `max_detectors` up to 32.
Discovery and state are republished after an MQTT reconnect.

The YAML deliberately sets ESPHome's ordinary MQTT entity discovery to false:
the bridge diagnostics arrive through the native API, while only the dynamic
detector devices use MQTT discovery. `log_topic` is also disabled to keep radio
diagnostic logs off MQTT.

### Alarm controls

The bridge device exposes native Home Assistant buttons for fire, CO and
combined interlink sound tests, fire/CO silence, checking the pairing state and
starting a 21-second pairing window. `Network Paired`, `Radio Command Running`
and `Last Radio Command` report command state and results. ESPHome device
availability, `Radio Initialized` and `Bridge Uptime` replace the original
serial heartbeat with explicit bridge and radio health reporting.

The sound-test buttons confirm only that the donor radio accepted and
transmitted the request. WiSafe2 detectors do not return individual results for
a remotely initiated sound test. To exercise and record the self-test result of
each detector, press that detector's physical test button. Emergency simulation
is intentionally not exposed.

## Tests

The radio packet decoder is platform-independent and has host-side coverage for
the observed status frames, tests, emergencies, silence, missing detectors,
battery/base flags and malformed input. Run it without an ESP32 toolchain:

```bash
./tests/run_tests.sh
```

GitHub Actions runs the protocol tests, validates the ESPHome configuration and
compiles the complete firmware for every push and pull request. CI copies the
tracked placeholder values from `secrets.yaml.example`; real credentials remain
in the ignored `secrets.yaml` file and are never required by the workflow.

## Versioning

The firmware/project version is set in `esphome.project.version` in
`wisafe2.yaml`. The ESPHome version pinned in `.github/requirements-ci.txt` is
the CI build toolchain version, not the project version. Release changes are
recorded in `CHANGELOG.md`; future releases should also use annotated Git tags
such as `v0.2.0`.

## Expected log

If the radio handshake works, the important result is:

```text
INIT TX: D3 19 50 00 7E
...
INIT RX: 46 7E
*** INIT OK: received expected 46 7E ***
```

If the first TX byte times out, check power, common ground, IRQ/CS wiring and the pin mapping before changing the protocol code.

## Current limitations

The standalone test harness intentionally does not implement Wi-Fi, heartbeat,
alarm entities, test commands or silence commands. Its pairing flow remains an
automatic boot-time bench test. The ESPHome implementation provides
Wi-Fi/API/OTA, packet decoding, persistent MQTT-discovered detector devices and
the supported management commands. WiSafe2 does not provide a known remote
command that individually self-tests every detector and reports its result.
