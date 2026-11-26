"""BenQ ScreenBar HALO 2 bridge over a Semtech LR1121 radio (ESPHome external component)."""

import esphome.codegen as cg
from esphome.components import spi
import esphome.config_validation as cv
from esphome import pins
from esphome.const import CONF_ADDRESS, CONF_ID, CONF_RESET_PIN

CODEOWNERS = ["@kunf"]
DEPENDENCIES = ["spi"]
AUTO_LOAD = []
MULTI_CONF = False

CONF_HALO2_ID = "halo2_id"
CONF_TRANSCEIVER = "transceiver"
CONF_BUSY_PIN = "busy_pin"
CONF_IRQ_PIN = "irq_pin"
CONF_POLL_INTERVAL = "poll_interval"
CONF_TCXO_VOLTAGE = "tcxo_voltage"

# SetTcxoMode voltage codes; "none" means the module has a plain crystal
TCXO_VOLTAGES = {
    "1.6V": 0, "1.7V": 1, "1.8V": 2, "2.2V": 3, "2.4V": 4, "2.7V": 5, "3.0V": 6, "3.3V": 7, "NONE": 0xFF, "AUTO": 0xFE,
}

halo2_ns = cg.esphome_ns.namespace("halo2")
Halo2Component = halo2_ns.class_("Halo2Component", cg.Component, spi.SPIDevice)
Lamp = halo2_ns.enum("Lamp", is_class=True)
SwitchKind = halo2_ns.enum("SwitchKind", is_class=True)
ButtonKind = halo2_ns.enum("ButtonKind", is_class=True)

TRANSCEIVERS = ["lr1121"]


def validate_address(value):
    """Four bytes in register order, the same as HALO2_ADDRESS in halo2_address.py."""
    if isinstance(value, str):
        parts = [p for p in value.replace(",", " ").split() if p]
        value = parts
    value = cv.ensure_list(cv.hex_uint8_t)(value)
    if len(value) != 4:
        raise cv.Invalid("address must be exactly 4 bytes, e.g. [0x4e, 0xf5, 0x5d, 0x43]")
    return value


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Halo2Component),
            cv.Optional(CONF_TRANSCEIVER, default="lr1121"): cv.one_of(*TRANSCEIVERS, lower=True),
            cv.Required(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_BUSY_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_ADDRESS): validate_address,
            cv.Optional(
                CONF_POLL_INTERVAL, default="5s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_TCXO_VOLTAGE, default="AUTO"): cv.one_of(
                *TCXO_VOLTAGES, upper=True, space=""
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)

HALO2_CHILD_SCHEMA = cv.Schema({cv.GenerateID(CONF_HALO2_ID): cv.use_id(Halo2Component)})


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_reset_pin(await cg.gpio_pin_expression(config[CONF_RESET_PIN])))
    cg.add(var.set_busy_pin(await cg.gpio_pin_expression(config[CONF_BUSY_PIN])))
    cg.add(var.set_irq_pin(await cg.gpio_pin_expression(config[CONF_IRQ_PIN])))
    cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL]))
    cg.add(var.set_tcxo_voltage_code(TCXO_VOLTAGES[config[CONF_TCXO_VOLTAGE]]))
    if CONF_ADDRESS in config:
        address = cg.RawExpression(
            "{" + ", ".join(f"0x{b:02X}" for b in config[CONF_ADDRESS]) + "}"
        )
        cg.add(var.set_address(address))
