import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv

from . import CONF_HALO2_ID, HALO2_CHILD_SCHEMA, SwitchKind, halo2_ns

DEPENDENCIES = ["halo2"]

Halo2Switch = halo2_ns.class_("Halo2Switch", switch.Switch)

CONF_POWER = "power"
CONF_ULTRASONIC = "ultrasonic"

# State always comes from the lamp, so never restore or send a state on boot
CONFIG_SCHEMA = cv.All(
    HALO2_CHILD_SCHEMA.extend(
        {
            cv.Optional(CONF_POWER): switch.switch_schema(
                Halo2Switch, icon="mdi:power", default_restore_mode="DISABLED"
            ),
            cv.Optional(CONF_ULTRASONIC): switch.switch_schema(
                Halo2Switch, icon="mdi:motion-sensor", default_restore_mode="DISABLED"
            ),
        }
    ),
    cv.has_at_least_one_key(CONF_POWER, CONF_ULTRASONIC),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HALO2_ID])
    for key, kind in ((CONF_POWER, SwitchKind.POWER), (CONF_ULTRASONIC, SwitchKind.ULTRASONIC)):
        if conf := config.get(key):
            sw = await switch.new_switch(conf, parent, kind)
            cg.add(parent.register_switch(kind, sw))
