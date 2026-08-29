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
detector, containing alarm, low-battery, base-problem, model, last-event,
last-test result, last-test timestamp and raw-frame entities. Unused detector
capacity is not advertised. The default inventory limit is 16 and can be changed with `max_detectors` up to 32.
Discovery and state are republished after an MQTT reconnect.

The YAML deliberately sets ESPHome's ordinary MQTT entity discovery to false:
the bridge diagnostics arrive through the native API, while only the dynamic
detector devices use MQTT discovery. `log_topic` is also disabled to keep radio
diagnostic logs off MQTT.

### Suggested Home Assistant alarm card

With the custom
[auto-entities](https://github.com/thomasloven/lovelace-auto-entities) and
[Mushroom](https://github.com/piitaya/lovelace-mushroom) cards installed, the
following dashboard card automatically adds one row for every detector exposed
by the bridge. It selects the detector's `Alarm` entity and obtains its
`Battery`, `Base`, `Last test result` and `Last test` entities from the same
Home Assistant device:

```yaml
type: custom:auto-entities
card:
  type: vertical-stack
card_param: cards
filter:
  include:
    - domain: binary_sensor
      device_manufacturer: FireAngel
      attributes:
        device_class: "/^(smoke|heat|carbon_monoxide)$/"
      options:
        type: custom:mushroom-template-card
        entity: this.entity_id
        primary: >-
          {% set dev = device_id(config.entity) %}
          {{ device_attr(dev, 'name_by_user') or device_attr(dev, 'name') }}
        secondary: >-
          {% set dev = device_id(config.entity) %}
          {% set ns = namespace(
            battery='unknown',
            base='unknown',
            test_result='unknown',
            last_test='unknown'
          ) %}

          {% for e in device_entities(dev) %}
            {% if state_attr(e, 'device_class') == 'battery' %}
              {% set ns.battery = states(e) %}
            {% elif state_attr(e, 'device_class') == 'problem' %}
              {% set ns.base = states(e) %}
            {% elif e.endswith('_test_result') %}
              {% set ns.test_result = states(e) %}
            {% elif state_attr(e, 'device_class') == 'timestamp' %}
              {% set ns.last_test = states(e) %}
            {% endif %}
          {% endfor %}

          {% if is_state(config.entity, 'on') %}
            ALARM
          {% elif ns.battery == 'on' %}
            Battery low
          {% elif ns.base == 'on' %}
            Base problem
          {% else %}
            Normal
          {% endif %}
          {% if ns.last_test not in ['unknown', 'unavailable', 'none', ''] %}
            · Last test {{ as_timestamp(ns.last_test)
              | timestamp_custom('%d %b %Y %H:%M', true) }}
            {% if ns.test_result not in ['unknown', 'unavailable', 'none', ''] %}
              ({{ ns.test_result }})
            {% endif %}
          {% else %}
            · No test recorded
          {% endif %}
        icon: |-
          {% if is_state(config.entity, 'on') %}
            mdi:fire-alert
          {% else %}
            mdi:smoke-detector
          {% endif %}
        color: >-
          {% set dev = device_id(config.entity) %}
          {% set ns = namespace(battery=false, base=false) %}

          {% for e in device_entities(dev) %}
            {% if state_attr(e, 'device_class') == 'battery'
                  and is_state(e, 'on') %}
              {% set ns.battery = true %}
            {% elif state_attr(e, 'device_class') == 'problem'
                    and is_state(e, 'on') %}
              {% set ns.base = true %}
            {% endif %}
          {% endfor %}

          {% if is_state(config.entity, 'on') %}
            red
          {% elif ns.battery or ns.base %}
            amber
          {% else %}
            green
          {% endif %}
sort:
  method: name
```

The filter uses the manufacturer and alarm device-class metadata published by
the component, so it does not depend on the ESPHome node name or generated
entity IDs. The last test result and UTC timestamp are retained by the firmware
and restored from flash after a reboot. The dashboard formats the timestamp in
Home Assistant's local timezone. A physical test received before SNTP has
synchronized still records its result, but cannot be assigned a reliable
timestamp.

## Alarm notification blueprint

The repository includes a Home Assistant automation blueprint for notifying on
real smoke, heat or carbon-monoxide alarms:

[FireAngel WiSafe2 real alarm notification](blueprints/automation/fireangel_wisafe2_alarm_notification.yaml)

It automatically covers detectors discovered after the automation is created.
A server-side `state_changed` trigger selects FireAngel alarm binary sensors by
manufacturer and device class, so renamed entities and the frontend registry
cache do not affect it. Only a transition of the detector's `Alarm` entity to
`on` is actionable; physical and remote test packets leave that entity off and
do not trigger notifications.

To install it, copy the blueprint to
`config/blueprints/automation/fireangel_wisafe2/` in Home Assistant, reload
automation blueprints, and create an automation from it. Its action selector can
send mobile-app notifications, persistent notifications, announcements or any
other Home Assistant action. Notification templates can use:

- `alarm_entity` and `alarm_entity_name`
- `alarm_device_id` and `alarm_device_name`
- `alarm_device_class` and the human-readable `alarm_type`
- `alarm_area_name` and `alarm_time`

The default action creates a persistent Home Assistant notification. Replace or
extend it when creating the automation to choose the desired recipients and
message.

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
battery/base flags, extended frames and malformed input. Known packet types use
minimum lengths because observed status and supervision variants carry trailing
fields that are not yet understood; decoding reads only the established fixed
offsets and preserves the complete raw frame for diagnostics. Run the tests
without an ESP32 toolchain:

```bash
./tests/run_tests.sh
```

GitHub Actions runs the protocol tests, validates the ESPHome configuration and
compiles the complete firmware for every push and pull request. CI copies the
tracked placeholder values from `secrets.yaml.example`; real credentials remain
in the ignored `secrets.yaml` file and are never required by the workflow.

The host suite validates packet encoding and decoding only. SPI-slave timing,
IRQ acknowledgement and the radio's occasional extra clocks require bench
testing with the donor radio and are not simulated by CI.

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
