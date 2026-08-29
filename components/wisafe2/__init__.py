import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, button, text_sensor, time
from esphome.const import (
    CONF_ID,
    CONF_TIME_ID,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["esp32", "mqtt", "time"]
AUTO_LOAD = ["binary_sensor", "button", "text_sensor"]

CONF_SCLK_PIN = "sclk_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_CS_PIN = "cs_pin"
CONF_IRQ_PIN = "irq_pin"
CONF_INITIALIZED = "initialized"
CONF_LAST_PACKET = "last_packet"
CONF_LAST_DEVICE = "last_device"
CONF_LAST_MODEL = "last_model"
CONF_LAST_EVENT = "last_event"
CONF_LAST_RESULT = "last_result"
CONF_LAST_BASE = "last_base"
CONF_LAST_BATTERY = "last_battery"
CONF_PAIRED = "paired"
CONF_COMMAND_BUSY = "command_busy"
CONF_LAST_COMMAND = "last_command"
CONF_SOUND_CO = "sound_co"
CONF_SOUND_FIRE = "sound_fire"
CONF_SOUND_COMBINED = "sound_combined"
CONF_SILENCE_CO = "silence_co"
CONF_SILENCE_FIRE = "silence_fire"
CONF_CHECK_PAIRING = "check_pairing"
CONF_START_PAIRING = "start_pairing"
CONF_DISCOVERY_PREFIX = "discovery_prefix"
CONF_MAX_DETECTORS = "max_detectors"
CONF_BRIDGE_DEVICE_ID = "bridge_device_id"
CONF_BRIDGE_MODEL_ID = "bridge_model_id"

wisafe2_ns = cg.esphome_ns.namespace("wisafe2")
WiSafe2Component = wisafe2_ns.class_("WiSafe2Component", cg.Component)
WiSafe2CommandButton = wisafe2_ns.class_("WiSafe2CommandButton", button.Button)
ManagementCommand = wisafe2_ns.enum("ManagementCommand", is_class=True)

_GPIO = cv.int_range(min=0, max=48)


def _device_id(value):
    value = cv.string_strict(value).strip().upper()
    if re.fullmatch(r"[0-9A-F]{6}", value) is None:
        raise cv.Invalid("bridge_device_id must be exactly six hexadecimal digits")
    return value


def _model_id(value):
    value = cv.string_strict(value).strip().upper()
    if re.fullmatch(r"[0-9A-F]{4}", value) is None:
        raise cv.Invalid("bridge_model_id must be exactly four hexadecimal digits")
    return value


def _validate_pins(config):
    pin_keys = [CONF_SCLK_PIN, CONF_MOSI_PIN, CONF_MISO_PIN, CONF_CS_PIN, CONF_IRQ_PIN]
    pins = [config[key] for key in pin_keys]
    if len(set(pins)) != len(pins):
        raise cv.Invalid("sclk_pin, mosi_pin, miso_pin, cs_pin and irq_pin must be different GPIOs")
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WiSafe2Component),
            cv.GenerateID(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Required(CONF_SCLK_PIN): _GPIO,
            cv.Required(CONF_MOSI_PIN): _GPIO,
            cv.Required(CONF_MISO_PIN): _GPIO,
            cv.Required(CONF_CS_PIN): _GPIO,
            cv.Required(CONF_IRQ_PIN): _GPIO,
            cv.Optional(CONF_DISCOVERY_PREFIX, default="homeassistant"): cv.string_strict,
            cv.Optional(CONF_MAX_DETECTORS, default=16): cv.int_range(min=1, max=32),
            cv.Optional(CONF_BRIDGE_DEVICE_ID, default="A5B813"): _device_id,
            cv.Optional(CONF_BRIDGE_MODEL_ID, default="1103"): _model_id,
            cv.Optional(CONF_INITIALIZED): binary_sensor.binary_sensor_schema(
                device_class="connectivity",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_PACKET): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:code-tags",
            ),
            cv.Optional(CONF_LAST_DEVICE): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:identifier",
            ),
            cv.Optional(CONF_LAST_MODEL): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:smoke-detector-variant",
            ),
            cv.Optional(CONF_LAST_EVENT): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:message-alert-outline",
            ),
            cv.Optional(CONF_LAST_RESULT): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:check-decagram-outline",
            ),
            cv.Optional(CONF_LAST_BASE): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:home-outline",
            ),
            cv.Optional(CONF_LAST_BATTERY): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:battery-outline",
            ),
            cv.Optional(CONF_PAIRED): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:access-point-check",
            ),
            cv.Optional(CONF_COMMAND_BUSY): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:progress-clock",
            ),
            cv.Optional(CONF_LAST_COMMAND): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:console-line",
            ),
            cv.Optional(CONF_SOUND_CO): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:molecule-co",
            ),
            cv.Optional(CONF_SOUND_FIRE): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:fire",
            ),
            cv.Optional(CONF_SOUND_COMBINED): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:alarm-light",
            ),
            cv.Optional(CONF_SILENCE_CO): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:volume-off",
            ),
            cv.Optional(CONF_SILENCE_FIRE): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:volume-off",
            ),
            cv.Optional(CONF_CHECK_PAIRING): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:access-point-check",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_START_PAIRING): button.button_schema(
                WiSafe2CommandButton,
                icon="mdi:access-point-plus",
                entity_category=ENTITY_CATEGORY_CONFIG,
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    cv.only_with_framework("esp-idf"),
    _validate_pins,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_pins(
        config[CONF_SCLK_PIN],
        config[CONF_MOSI_PIN],
        config[CONF_MISO_PIN],
        config[CONF_CS_PIN],
        config[CONF_IRQ_PIN],
    ))
    cg.add(var.set_discovery_prefix(config[CONF_DISCOVERY_PREFIX]))
    cg.add(var.set_max_detectors(config[CONF_MAX_DETECTORS]))
    cg.add(var.set_bridge_device_id(int(config[CONF_BRIDGE_DEVICE_ID], 16)))
    cg.add(var.set_bridge_model_id(int(config[CONF_BRIDGE_MODEL_ID], 16)))
    clock = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(clock))

    if initialized_config := config.get(CONF_INITIALIZED):
        initialized = await binary_sensor.new_binary_sensor(initialized_config)
        cg.add(var.set_initialized_sensor(initialized))

    if last_packet_config := config.get(CONF_LAST_PACKET):
        last_packet = await text_sensor.new_text_sensor(last_packet_config)
        cg.add(var.set_last_packet_sensor(last_packet))

    for key, setter in (
        (CONF_LAST_DEVICE, "set_last_device_sensor"),
        (CONF_LAST_MODEL, "set_last_model_sensor"),
        (CONF_LAST_EVENT, "set_last_event_sensor"),
        (CONF_LAST_RESULT, "set_last_result_sensor"),
        (CONF_LAST_BASE, "set_last_base_sensor"),
        (CONF_LAST_BATTERY, "set_last_battery_sensor"),
    ):
        if sensor_config := config.get(key):
            sensor = await text_sensor.new_text_sensor(sensor_config)
            cg.add(getattr(var, setter)(sensor))

    for key, setter in (
        (CONF_PAIRED, "set_paired_sensor"),
        (CONF_COMMAND_BUSY, "set_command_busy_sensor"),
    ):
        if sensor_config := config.get(key):
            sensor = await binary_sensor.new_binary_sensor(sensor_config)
            cg.add(getattr(var, setter)(sensor))

    if last_command_config := config.get(CONF_LAST_COMMAND):
        sensor = await text_sensor.new_text_sensor(last_command_config)
        cg.add(var.set_last_command_sensor(sensor))

    for key, command in (
        (CONF_SOUND_CO, ManagementCommand.SOUND_CO),
        (CONF_SOUND_FIRE, ManagementCommand.SOUND_FIRE),
        (CONF_SOUND_COMBINED, ManagementCommand.SOUND_COMBINED),
        (CONF_SILENCE_CO, ManagementCommand.SILENCE_CO),
        (CONF_SILENCE_FIRE, ManagementCommand.SILENCE_FIRE),
        (CONF_CHECK_PAIRING, ManagementCommand.QUERY_PAIRING),
        (CONF_START_PAIRING, ManagementCommand.START_PAIRING),
    ):
        if button_config := config.get(key):
            command_button = await button.new_button(button_config)
            cg.add(command_button.set_parent(var))
            cg.add(command_button.set_command(command))
