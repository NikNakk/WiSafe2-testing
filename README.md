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
last-test result, last-test timestamp, raw-frame and remote-radio diagnostic
entities, including whether each retained detector is present in the latest
network SID map. Live `71` status frames additionally expose calibrated,
generic device-fault, on-base, detector-battery-fault, AC-failure and
radio-module-battery-fault states, plus the complete raw status byte. The bridge
walks the radio's SID map to identify paired detectors and also reacts
immediately when the radio announces a newly paired SID. Each detector also
exposes Home Assistant device triggers for alarm
detected, alarm cleared, physical test passed and physical test failed. These
events are not retained, so reconnecting Home Assistant or the broker cannot
replay an old alarm or test as a new trigger. Alarm
state remains unknown until live alarm-on (`50`) or alarm-off (`51`) traffic is
received because the diagnostic query cannot report an alarm already in
progress. The optional `refresh_detectors` button queries diagnostic state from
all known detectors on demand, first reconciling the radio's current SID map.
This remote diagnostic query is deliberately not scheduled periodically to
avoid unnecessary radio traffic and battery use;
only the bridge diagnostic and SID map are polled every 60 seconds. Unused
detector capacity is not advertised. The default inventory
limit is 16 and can be changed with `max_detectors` up to 32.
Discovery and state are republished after an MQTT reconnect.

Remote diagnostics expose the two battery readings and diagnostic flags exactly
as raw values. They do not contain an established on-base field, and neither
reference implementation documents reliable thresholds that turn the battery
bytes into an OK/low state. Consequently, `Base` and `Battery` remain unknown
until a live `71` status packet supplies those states. Treating zero diagnostic
flags as proof that both states are OK would hide genuine faults if the flags
have a different meaning on another detector model. The two raw battery
readings, raw RSSI and radio fault count are published as measurement sensors so
Home Assistant records long-term statistics and can graph their history.

The YAML deliberately sets ESPHome's ordinary MQTT entity discovery to false:
the bridge diagnostics arrive through the native API, while only the dynamic
detector devices use MQTT discovery. `log_topic` is also disabled to keep radio
diagnostic logs off MQTT. On ESP-IDF, `idf_send_async` keeps broker I/O on a
separate task so a slow MQTT publish cannot block ESPHome's main loop while a
detector diagnostic packet is being processed. The component also processes
radio events in bounded batches, yielding between bursts without affecting the
dedicated SPI task.

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
            battery_low=false,
            problem=false,
            test_result='unknown',
            last_test='unknown'
          ) %}

          {% for e in device_entities(dev) %}
            {% if state_attr(e, 'device_class') == 'battery'
                  and is_state(e, 'on') %}
              {% set ns.battery_low = true %}
            {% elif state_attr(e, 'device_class') == 'problem'
                    and is_state(e, 'on') %}
              {% set ns.problem = true %}
            {% elif e.endswith('_test_result') %}
              {% set ns.test_result = states(e) %}
            {% elif state_attr(e, 'device_class') == 'timestamp' %}
              {% set ns.last_test = states(e) %}
            {% endif %}
          {% endfor %}

          {% if is_state(config.entity, 'on') %}
            ALARM
          {% elif ns.battery_low %}
            Battery low
          {% elif ns.problem %}
            Detector problem
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
          {% set ns = namespace(battery=false, problem=false) %}

          {% for e in device_entities(dev) %}
            {% if state_attr(e, 'device_class') == 'battery'
                  and is_state(e, 'on') %}
              {% set ns.battery = true %}
            {% elif state_attr(e, 'device_class') == 'problem'
                    and is_state(e, 'on') %}
              {% set ns.problem = true %}
            {% endif %}
          {% endfor %}

          {% if is_state(config.entity, 'on') %}
            red
          {% elif ns.battery or ns.problem %}
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

### Notification blueprints

The same live alarm and physical-test traffic is also available through each
detector's device triggers in Home Assistant's automation editor. The blueprints
below remain useful when one automation should automatically include every
current and future FireAngel detector.

#### Real alarm notifications

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

#### Detector test notifications

A companion blueprint runs actions whenever a detector reports a new physical
test result:

[FireAngel WiSafe2 detector test notification](blueprints/automation/fireangel_wisafe2_test_notification.yaml)

It triggers from the persisted `Last test` timestamp rather than the PASS/FAIL
state, so two consecutive tests with the same result are both detected. A short,
configurable delay lets the companion result and event entities update before
the actions run. A five-minute freshness check prevents an old retained test
from generating a notification after a routine Home Assistant restart.
Templates can use:

- `test_device_id`, `test_device_name`, and `test_area_name`
- `test_alarm_device_class` and the human-readable `test_alarm_type`
- `test_result` and `test_event`
- `test_entity`, `test_timestamp`, and locally formatted `test_time`

Install this blueprint in the same Home Assistant blueprint directory described
above. It covers physical detector tests. The bridge's remote sound-test
buttons confirm radio transmission but do not generate per-detector test results,
so they cannot activate it.

### Alarm controls

The bridge device exposes native Home Assistant buttons for fire, CO and
combined interlink sound tests, fire/CO silence, checking the pairing state and
starting a 21-second pairing window. `Network Paired`, `Radio Command Running`
and `Last Radio Command` report command state and results. ESPHome device
availability, `Radio Initialized` and `Bridge Uptime` replace the original
serial heartbeat with explicit bridge and radio health reporting.

The component also polls the attached module's local diagnostic state and
network SID map every 60 seconds, and responds whenever the radio requests the
bridge identity with `41 7E`. These are local SPI management exchanges rather
than periodic per-detector queries or synthetic alarm-network heartbeats.
At the radio boundary, payload bytes `7E` and `7D` use WiSafe2 byte stuffing
(`7D 01` and `7D 02` respectively); the terminal `7E` remains unescaped. Frames
are restored to their logical bytes before decoding and diagnostic publishing.

The sound-test buttons confirm only that the donor radio accepted and
transmitted the request. WiSafe2 detectors do not return individual results for
a remotely initiated sound test. To exercise and record the self-test result of
each detector, press that detector's physical test button. Emergency simulation
is intentionally not exposed.

## Comparison with the reference implementations

This project draws on two independent, reverse-engineered implementations:
[C19HOP/WiSafe2-to-HomeAssistant-Bridge](https://github.com/C19HOP/WiSafe2-to-HomeAssistant-Bridge)
and [Tho85/ws2mqtt](https://github.com/Tho85/ws2mqtt). Neither is an official
WiSafe2 protocol specification, although C19HOP's repository includes
[captured debug output](https://github.com/C19HOP/WiSafe2-to-HomeAssistant-Bridge/blob/master/FireAngelProConnectedGateway/DeviceTest2.txt)
from FireAngel's own Connected Gateway firmware. Where the two implementations
disagree, that manufacturer-firmware evidence is preferred.

| Area | C19HOP bridge | ws2mqtt | This ESPHome component |
|---|---|---|---|
| Hardware and transport | Arduino Nano acting as the SPI slave, with a USB serial connection to Home Assistant. | Arduino/ATmega SPI-to-UART adapter plus a separate ESP32 MQTT gateway. | ESP32-S3 talks to the radio directly as an SPI slave and runs ESPHome, eliminating the intermediate microcontroller and serial protocol. |
| Detector inventory | Detector IDs are learned from received traffic and configured manually in the supplied Home Assistant templates. | Maintains a device database, walks the `D4 03` SID map, resolves unknown SIDs with `D3 06 <sid> 01`, and reacts to `D4 09` join notifications. | Uses the ws2mqtt-style SID discovery flow, persists the resulting inventory, and creates detector devices using MQTT discovery. |
| Remote diagnostics | Primarily derives detector state from unsolicited network traffic. | Supports `D3 06 <sid> 00` remote diagnostic requests, although its README recommends avoiding routine active queries to conserve detector batteries. | Queries a detector while discovering it and exposes an explicit **Refresh Detector Diagnostics** button. Remote requests are serialized and may take tens of seconds because the radio returns `C4`/`D4 06` asynchronously. It does not periodically poll individual detectors; only the attached radio and SID map are polled every 60 seconds. |
| `71` status flags | The original published analysis treats `04` as docked/on-base and `02` or `40` as low-battery indications. Its captured official-gateway log instead names bits `01` calibrated, `02` faulty, `04` on-base, `08` detector-battery fault, `10` AC failed and `20` radio-module-battery fault. | Exposes generic fault (`02`), docked (`04`), detector battery (`08`) and radio-module battery (`20`) binary sensors. | Uses all six names from the FireAngel gateway log. Separate binary sensors expose each meaning, `Battery` remains a compatibility OR of `08` and `20`, `Base` remains the inverse of `04`, and the raw byte preserves unnamed bits `40` and `80`. |
| Alarm state | Focuses on event reporting through serial/Home Assistant templates. | Handles live alarm-on and alarm-off traffic and deliberately starts alarm state as unknown because it cannot be queried. | Likewise changes alarm state only from live `50`/`51` events. Diagnostic polling never assumes that an alarm is off. |
| Reserved-byte framing | The available SPI analysis does not describe a separate byte-stuffing layer. | Explicitly maps payload `7E` to `7D 01` and payload `7D` to `7D 02`. | Applies the ws2mqtt mapping on every SPI transmit and receive path while leaving the final `7E` delimiter unescaped. |
| Inventory removal | Has no comparable persistent automatic database to reconcile. | Clears its device database when the attached radio reports the unpaired/reset SID state (`D2` SID `40`). | Deliberately retains previously discovered detectors. A per-detector `Network member` diagnostic shows whether each retained entry is present in the latest SID map, avoiding silent Home Assistant device deletion after a transient reset or radio replacement. |
| Trailing packet fields | Unknown trailing bytes are generally passed through or ignored. | Some receive structures label trailing bytes as SID and sequence metadata. | Accepts extended frames but reads only independently established offsets. The possible SID/sequence fields remain undecoded until captures confirm their meaning across packet types. |
| Command pacing | Timing follows the Nano firmware's blocking SPI/IRQ flow. | Enforces a minimum interval of approximately 500 ms between transmissions. | Enforces a 500 ms quiet interval after transmitted and received frames before issuing another command. This also accommodates asynchronous `C4` identity and `D4 06` status replies observed on real hardware. |
| Model catalogue | Documents the FP2620W2, FP1720W2, WST-630, W2-SVP-630 and W2-CO-10X devices used by that project. | Also reports testing with the ST-630-DE(P) and HT-630-EUT, and stores model numbers in manufacturer numeric order such as `08ED`. | Includes the C19HOP models plus ST-630-DE(P). Internal constants and raw entities retain packet-byte order (`ED08`, `7C04`) for compatibility; FireAngel's gateway displays those numeric models as `08ED`, `047C`. The HT-630-EUT remains identified by its reported device type until an explicit model ID is available. |
| Home Assistant model | Serial JSON plus user-maintained template sensors and commands. | MQTT discovery devices and gateway-level Home Assistant events. | ESPHome native bridge controls plus dynamically discovered per-detector MQTT entities, per-device alarm/test triggers and network-wide automation blueprints. `via_device` is intentionally omitted: the bridge belongs to ESPHome's native-API config entry while the detectors belong to MQTT, and linking across integrations would require a duplicate MQTT bridge device. |

The main open protocol questions are therefore the meaning of `71` bits `40`
and `80`, trailing SID/sequence bytes, and model IDs not yet seen in this
project's captures. These should be resolved with timestamped raw frames and,
where timing is involved, logic-analyser traces from real hardware.

## Tests

The radio packet decoder is platform-independent and has host-side coverage for
the observed detector status frames, tests, emergencies, silence, attached-radio
diagnostics, identity responses, battery/base flags, extended frames and
malformed input, including reserved-byte escaping. `D2` is treated as a
diagnostic response from the attached radio rather than a missing-detector
event. Known packet types use minimum
lengths because observed variants carry trailing fields that are not yet
understood; decoding reads only the established fixed offsets and preserves the
complete raw frame for diagnostics. Run the tests without an ESP32 toolchain:

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
