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

5. Enters raw receive mode and prints bytes/packets ending in `7E`.

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

This intentionally does not yet implement Wi-Fi, ESPHome, packet decoding, pairing, heartbeat, alarm entities, test commands or silence commands. Those should be added only after raw radio communication is demonstrated reliably.
