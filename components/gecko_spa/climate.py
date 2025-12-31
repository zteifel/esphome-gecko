import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID
from . import gecko_spa_ns, GeckoSpa

DEPENDENCIES = ["gecko_spa"]

GeckoSpaClimate = gecko_spa_ns.class_(
    "GeckoSpaClimate", climate.Climate, cg.Component
)

CONF_GECKO_SPA_ID = "gecko_spa_id"

CONFIG_SCHEMA = climate._CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(GeckoSpaClimate),
        cv.GenerateID(CONF_GECKO_SPA_ID): cv.use_id(GeckoSpa),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_GECKO_SPA_ID])
    var = cg.new_Pvariable(config[CONF_ID], parent)
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    cg.add(parent.set_climate(var))
