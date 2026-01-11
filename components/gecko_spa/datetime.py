import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import datetime
from esphome.const import CONF_ID
from . import gecko_spa_ns, GeckoSpa

DEPENDENCIES = ["gecko_spa"]

CONF_GECKO_SPA_ID = "gecko_spa_id"
CONF_SENSOR_TYPE = "type"

DATETIME_TYPES = {
    "spa_time": "SPA_TIME",
}
DATE_TYPES = {
    "rinse_filter": "RINSE_FILTER",
    "clean_filter": "CLEAN_FILTER",
    "change_water": "CHANGE_WATER",
    "spa_checkup": "SPA_CHECKUP",
}

GeckoSpaDateTime = gecko_spa_ns.class_("GeckoSpaDateTime", datetime.DateTimeEntity, cg.Component)
GeckoSpaDate = gecko_spa_ns.class_("GeckoSpaDate", datetime.DateEntity, cg.Component)

_BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_GECKO_SPA_ID): cv.use_id(GeckoSpa),
        # cv.Required(CONF_SENSOR_TYPE): cv.enum(SENSOR_TYPES, lower=True),
    }
)

_BASE_DATETIME_SCHEMA = _BASE_SCHEMA.extend(
    {
        cv.Required(CONF_SENSOR_TYPE): cv.enum(DATETIME_TYPES, lower=True),
    }
)

_BASE_DATE_SCHEMA = _BASE_SCHEMA.extend(
    {
        cv.Required(CONF_SENSOR_TYPE): cv.enum(DATE_TYPES, lower=True),
    }
)

CONFIG_SCHEMA = cv.Any(
    datetime.datetime_schema(GeckoSpaDateTime).extend(_BASE_DATETIME_SCHEMA),
    datetime.date_schema(GeckoSpaDate).extend(_BASE_DATE_SCHEMA),
)


async def to_code(config):
    # "type" is a reservet config key for datetime entities, need to pull value and adjust config accordingly
    sensor_type = config[CONF_SENSOR_TYPE]
    if sensor_type in DATETIME_TYPES.keys():
        config[CONF_SENSOR_TYPE] = "DATETIME"
    elif sensor_type in DATE_TYPES.keys():
        config[CONF_SENSOR_TYPE] = "DATE"
    else:
        raise ValueError(f"Unsupported sensor type: {sensor_type}")

    parent = await cg.get_variable(config[CONF_GECKO_SPA_ID])
    var = await datetime.new_datetime(config)

    if sensor_type == "spa_time":
        cg.add(parent.set_spa_time_datetime(var))
    elif sensor_type == "rinse_filter":
        cg.add(parent.set_rinse_filter_date(var))
    elif sensor_type == "clean_filter":
        cg.add(parent.set_clean_filter_date(var))
    elif sensor_type == "change_water":
        cg.add(parent.set_change_water_date(var))
    elif sensor_type == "spa_checkup":
        cg.add(parent.set_spa_checkup_date(var))
    else:
        raise ValueError(f"Unsupported sensor type: {sensor_type}")