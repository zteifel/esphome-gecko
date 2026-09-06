import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import CONF_ID, CONF_ADDRESS, CONF_SDA, CONF_SCL
from esphome.core import CORE

AUTO_LOAD = ["climate", "switch", "select", "binary_sensor", "text_sensor", "sensor"]

CONF_UART_ID = "uart_id"
CONF_RESET_PIN = "reset_pin"
CONF_NOTIF_DATE_FORMAT = "notif_date_format"

gecko_spa_ns = cg.esphome_ns.namespace("gecko_spa")
GeckoSpa = gecko_spa_ns.class_("GeckoSpa", cg.Component)

NotifDateFormat = gecko_spa_ns.enum("NotifDateFormat", is_class=True)
NOTIF_DATE_FORMATS = {
    "Y-M-D": NotifDateFormat.Y_M_D,
    "D-M-Y": NotifDateFormat.D_M_Y,
}

DEFAULT_I2C_ADDRESS = 0x17


def _validate_transport(config):
    has_uart = CONF_UART_ID in config
    has_i2c = CONF_SDA in config or CONF_SCL in config or CONF_ADDRESS in config

    if has_uart and has_i2c:
        raise cv.Invalid(
            f"gecko_spa supports exactly one transport at a time - specify either "
            f"'{CONF_UART_ID}' (Arduino I2C-proxy over UART) or '{CONF_SDA}'/'{CONF_SCL}' "
            f"(direct I2C), not both."
        )
    if not has_uart and not has_i2c:
        raise cv.Invalid(
            f"gecko_spa requires a transport - specify either '{CONF_UART_ID}' "
            f"(Arduino I2C-proxy over UART) or '{CONF_SDA}' and '{CONF_SCL}' (direct I2C)."
        )
    if has_i2c and (CONF_SDA not in config or CONF_SCL not in config):
        raise cv.Invalid(f"Direct I2C transport requires both '{CONF_SDA}' and '{CONF_SCL}'.")
    if has_i2c and CORE.target_framework != "esp-idf":
        raise cv.Invalid(
            "Direct I2C transport requires 'framework: esp-idf' under esp32: - the raw "
            "I2C driver it uses (to toggle slave/master roles) isn't available under "
            "framework: arduino. Use the UART transport (uart_id:) instead, or switch "
            "frameworks."
        )
    return config


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(GeckoSpa),
            # Transport A: Arduino I2C-proxy over UART (the original design).
            cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            # Transport B: direct I2C. Plain pin numbers, not a full i2c: bus -
            # this component owns the port directly (raw ESP-IDF driver,
            # toggling slave/master roles), which esphome's i2c component
            # doesn't support, and it can't be shared with an i2c: platform
            # device. Requires framework: esp-idf - enforced above.
            cv.Optional(CONF_SDA): pins.internal_gpio_pin_number,
            cv.Optional(CONF_SCL): pins.internal_gpio_pin_number,
            cv.Optional(CONF_ADDRESS): cv.i2c_address,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_NOTIF_DATE_FORMAT, default="D-M-Y"): cv.enum(NOTIF_DATE_FORMATS, upper=True),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _validate_transport,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    if CONF_UART_ID in config:
        # ESPHome only copies a component's source files into the build when
        # that component actually appears in the config - since uart: is
        # optional here, gecko_spa.h/.cpp must not reference uart's types at
        # all unless this define says the user actually configured it.
        cg.add_define("USE_GECKO_SPA_UART")
        uart_component = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart_parent(uart_component))
    else:
        cg.add(var.set_i2c_sda_pin(config[CONF_SDA]))
        cg.add(var.set_i2c_scl_pin(config[CONF_SCL]))
        cg.add(var.set_i2c_address(config.get(CONF_ADDRESS, DEFAULT_I2C_ADDRESS)))

    if CONF_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(pin))

    if CONF_NOTIF_DATE_FORMAT in config:
        cg.add(var.set_notif_date_format(config[CONF_NOTIF_DATE_FORMAT]))
