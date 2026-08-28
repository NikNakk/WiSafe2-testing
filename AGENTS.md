# Repository guidance

## Scope

This repository contains two related implementations:

- `main/` is the standalone ESP-IDF radio test harness.
- `components/wisafe2/` is the ESPHome external component used by
  `wisafe2.yaml`.

The WiSafe2 radio is the SPI master and the ESP32-S3 is the SPI slave. Preserve
the IRQ handshake and byte-level timing unless a hardware test demonstrates a
safe change.

## Validation

Run the host protocol tests after changing packet decoding:

```sh
./tests/run_tests.sh
```

For component or configuration changes, also validate and compile the ESPHome
firmware:

```sh
esphome config wisafe2.yaml
esphome compile wisafe2.yaml
```

## Secrets and generated files

- Never commit `secrets.yaml` or print its values in logs or test output.
- Keep safe placeholders in `secrets.yaml.example` when adding a required
  secret.
- Do not commit `.esphome/`, `build/`, `sdkconfig`, or `eim_config.toml`.

## Versions and changelog

- The firmware/project version is `esphome.project.version` in
  `wisafe2.yaml`. Update it when preparing a release.
- Record user-visible changes in `CHANGELOG.md` under `Unreleased`, then move
  them into the matching version section when releasing.
- `.github/requirements-ci.txt` pins the ESPHome build toolchain version; it is
  not the firmware/project version.
- Use annotated Git tags named `vMAJOR.MINOR.PATCH` for future releases.

## Safety

This integration is diagnostic and automation software, not part of the
alarms' certified life-safety function. Do not describe it as a replacement for
the FireAngel alarm system or its required testing and maintenance.
