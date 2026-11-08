import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import time as time_, uart
from esphome.const import CONF_ID


def _validate_clock_sync(config):
    if config.get(CONF_CLOCK_SYNC) and CONF_TIME_ID not in config:
        raise cv.Invalid("clock_sync requires time_id")
    return config

DEPENDENCIES = ["uart"]
AUTO_LOAD = ["climate", "switch", "select", "binary_sensor", "text_sensor", "sensor"]

CONF_UART_ID = "uart_id"
CONF_RESET_PIN = "reset_pin"
CONF_NOTIF_DATE_FORMAT = "notif_date_format"
CONF_SERIAL_PROXY_ID = "serial_proxy_id"
CONF_TIME_ID = "time_id"
CONF_CLOCK_SYNC = "clock_sync"

gecko_spa_ns = cg.esphome_ns.namespace("gecko_spa")
GeckoSpa = gecko_spa_ns.class_("GeckoSpa", cg.Component, uart.UARTDevice)

NotifDateFormat = gecko_spa_ns.enum("NotifDateFormat", is_class=True)
NOTIF_DATE_FORMATS = {
    "Y-M-D": NotifDateFormat.Y_M_D,
    "D-M-Y": NotifDateFormat.D_M_Y,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GeckoSpa),
            cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_NOTIF_DATE_FORMAT, default="Y-M-D"): cv.enum(NOTIF_DATE_FORMATS, upper=True),
            # Time source for setting the spa clock; clock_sync syncs automatically on drift > 2 min
            cv.Optional(CONF_TIME_ID): cv.use_id(time_.RealTimeClock),
            cv.Optional(CONF_CLOCK_SYNC, default=False): cv.boolean,
            # Optional serial_proxy on the same UART, used to reflash the Arduino over WiFi
            cv.Optional(CONF_SERIAL_PROXY_ID): cv.use_id(
                cg.esphome_ns.namespace("serial_proxy").class_("SerialProxy")
            ),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_clock_sync,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_component = await cg.get_variable(config[CONF_UART_ID])
    cg.add(var.set_uart_parent(uart_component))

    if CONF_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(pin))

    if CONF_NOTIF_DATE_FORMAT in config:
        cg.add(var.set_notif_date_format(config[CONF_NOTIF_DATE_FORMAT]))

    if CONF_SERIAL_PROXY_ID in config:
        proxy = await cg.get_variable(config[CONF_SERIAL_PROXY_ID])
        cg.add(var.set_serial_proxy(proxy))

    if CONF_TIME_ID in config:
        t = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time(t))
    cg.add(var.set_clock_sync(config[CONF_CLOCK_SYNC]))
