import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_STATUS, ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_HALO2_ID, HALO2_CHILD_SCHEMA

DEPENDENCIES = ["halo2"]

CONFIG_SCHEMA = cv.All(
    HALO2_CHILD_SCHEMA.extend(
        {
            cv.Optional(CONF_ADDRESS): text_sensor.text_sensor_schema(
                icon="mdi:identifier", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
            cv.Optional(CONF_STATUS): text_sensor.text_sensor_schema(
                icon="mdi:radio-tower", entity_category=ENTITY_CATEGORY_DIAGNOSTIC
            ),
        }
    ),
    cv.has_at_least_one_key(CONF_ADDRESS, CONF_STATUS),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_HALO2_ID])
    if conf := config.get(CONF_ADDRESS):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.set_address_text_sensor(sens))
    if conf := config.get(CONF_STATUS):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(parent.set_status_text_sensor(sens))
