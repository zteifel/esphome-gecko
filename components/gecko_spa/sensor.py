import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import ICON_TIMER, STATE_CLASS_TOTAL_INCREASING, UNIT_MINUTE
from . import GeckoSpa

DEPENDENCIES = ["gecko_spa"]

CONF_GECKO_SPA_ID = "gecko_spa_id"
CONF_SENSOR_TYPE = "type"

BASE_SCHEMA = cv.Schema({cv.GenerateID(CONF_GECKO_SPA_ID): cv.use_id(GeckoSpa)})

CONFIG_SCHEMA = cv.typed_schema(
    {
        "pump_timer": sensor.sensor_schema(
            unit_of_measurement=UNIT_MINUTE,
            icon=ICON_TIMER,
            accuracy_decimals=0,
        ).extend(BASE_SCHEMA),
        # Times the Arduino proxy stopped answering and was reset (since ESP boot)
        "arduino_freezes": sensor.sensor_schema(
            icon="mdi:snowflake-alert",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ).extend(BASE_SCHEMA),
    },
    key=CONF_SENSOR_TYPE,
    lower=True,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_GECKO_SPA_ID])
    var = await sensor.new_sensor(config)

    sensor_type = config[CONF_SENSOR_TYPE]
    if sensor_type == "pump_timer":
        cg.add(parent.set_pump_timer_sensor(var))
    elif sensor_type == "arduino_freezes":
        cg.add(parent.set_arduino_freezes_sensor(var))
