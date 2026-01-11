import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import CONF_ID
from . import gecko_spa_ns, GeckoSpa

DEPENDENCIES = ["gecko_spa"]

CONF_GECKO_SPA_ID = "gecko_spa_id"
CONF_SENSOR_TYPE = "type"

SENSOR_TYPES = {
    "spa_time": "SPA_TIME",
    "rinse_filter": "RINSE_FILTER",
    "clean_filter": "CLEAN_FILTER",
    "change_water": "CHANGE_WATER",
    "spa_checkup": "SPA_CHECKUP",
}

CONFIG_SCHEMA = text_sensor.text_sensor_schema().extend(
    {
        cv.GenerateID(CONF_GECKO_SPA_ID): cv.use_id(GeckoSpa),
        cv.Required(CONF_SENSOR_TYPE): cv.enum(SENSOR_TYPES, lower=True),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_GECKO_SPA_ID])
    var = await text_sensor.new_text_sensor(config)

    sensor_type = config[CONF_SENSOR_TYPE]
    if sensor_type == "spa_time":
        cg.add(parent.set_spa_time_text_sensor(var))
    elif sensor_type == "rinse_filter":
        cg.add(parent.set_rinse_filter_text_sensor(var))
    elif sensor_type == "clean_filter":
        cg.add(parent.set_clean_filter_text_sensor(var))
    elif sensor_type == "change_water":
        cg.add(parent.set_change_water_text_sensor(var))
    elif sensor_type == "spa_checkup":
        cg.add(parent.set_spa_checkup_text_sensor(var))