import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import CONF_HALO2_ID, HALO2_CHILD_SCHEMA, ButtonKind, halo2_ns

DEPENDENCIES = ["halo2"]

Halo2Button = halo2_ns.class_("Halo2Button", button.Button)

CONF_AUTO = "auto"
CONF_DISCOVER_ADDRESS = "discover_address"

CONFIG_SCHEMA = cv.All(
    HALO2_CHILD_SCHEMA.extend(
        {
            cv.Optional(CONF_AUTO): button.button_schema(Halo2Button, icon="mdi:auto-mode"),
            cv.Optional(CONF_DISCOVER_ADDRESS): button.button_schema(
                Halo2Button, icon="mdi:radar", entity_category=ENTITY_CATEGORY_CONFIG
            ),
        }
    ),
    cv.has_at_least_one_key(CONF_AUTO, CONF_DISCOVER_ADDRESS),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HALO2_ID])
    for key, kind in ((CONF_AUTO, ButtonKind.AUTO), (CONF_DISCOVER_ADDRESS, ButtonKind.DISCOVER)):
        if conf := config.get(key):
            await button.new_button(conf, parent, kind)
