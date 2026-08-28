import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, text_sensor
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

DEPENDENCIES = ["esp32"]
AUTO_LOAD = ["binary_sensor", "text_sensor"]

CONF_SCLK_PIN = "sclk_pin"
CONF_MOSI_PIN = "mosi_pin"
CONF_MISO_PIN = "miso_pin"
CONF_CS_PIN = "cs_pin"
CONF_IRQ_PIN = "irq_pin"
CONF_INITIALIZED = "initialized"
CONF_LAST_PACKET = "last_packet"

wisafe2_ns = cg.esphome_ns.namespace("wisafe2")
WiSafe2Component = wisafe2_ns.class_("WiSafe2Component", cg.Component)

_GPIO = cv.int_range(min=0, max=48)


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
            cv.Required(CONF_SCLK_PIN): _GPIO,
            cv.Required(CONF_MOSI_PIN): _GPIO,
            cv.Required(CONF_MISO_PIN): _GPIO,
            cv.Required(CONF_CS_PIN): _GPIO,
            cv.Required(CONF_IRQ_PIN): _GPIO,
            cv.Optional(CONF_INITIALIZED): binary_sensor.binary_sensor_schema(
                device_class="connectivity",
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_PACKET): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:code-tags",
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

    if initialized_config := config.get(CONF_INITIALIZED):
        initialized = await binary_sensor.new_binary_sensor(initialized_config)
        cg.add(var.set_initialized_sensor(initialized))

    if last_packet_config := config.get(CONF_LAST_PACKET):
        last_packet = await text_sensor.new_text_sensor(last_packet_config)
        cg.add(var.set_last_packet_sensor(last_packet))
