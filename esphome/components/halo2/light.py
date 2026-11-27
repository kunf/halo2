import esphome.codegen as cg
from esphome.components import light
import esphome.config_validation as cv
from esphome.const import CONF_DEFAULT_TRANSITION_LENGTH, CONF_OUTPUT_ID

from . import CONF_HALO2_ID, HALO2_CHILD_SCHEMA, Lamp, halo2_ns

DEPENDENCIES = ["halo2"]

Halo2Light = halo2_ns.class_("Halo2Light", light.LightOutput)

CONF_FRONT = "front"
CONF_BACK = "back"

# The lamp fades by itself, and its state comes from the lamp: no ESPHome transitions,
# no restoring an old state on boot.
_LIGHT_SCHEMA = light.light_schema(
    Halo2Light, light.LightType.BRIGHTNESS_ONLY, default_restore_mode="ALWAYS_OFF"
).extend(
    {
        cv.Optional(
            CONF_DEFAULT_TRANSITION_LENGTH, default="0s"
        ): cv.positive_time_period_milliseconds,
    }
)

CONFIG_SCHEMA = cv.All(
    HALO2_CHILD_SCHEMA.extend(
        {
            cv.Optional(CONF_FRONT): _LIGHT_SCHEMA,
            cv.Optional(CONF_BACK): _LIGHT_SCHEMA,
        }
    ),
    cv.has_at_least_one_key(CONF_FRONT, CONF_BACK),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HALO2_ID])
    for key, lamp in ((CONF_FRONT, Lamp.FRONT), (CONF_BACK, Lamp.BACK)):
        if conf := config.get(key):
            output = cg.new_Pvariable(conf[CONF_OUTPUT_ID], parent, lamp)
            await light.register_light(output, conf)
            cg.add(parent.register_light(lamp, output))
