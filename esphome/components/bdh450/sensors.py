import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_CLOCK_PIN,
    CONF_DATA_PIN,
    CONF_STROBE_PIN,
    CONF_HUMIDITY,
    DEVICE_CLASS_HUMIDITY,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)
from esphome.cpp_helpers import gpio_pin_expression

bdh450_ns = cg.esphome_ns.namespace("bdh450")
BDH450Sensor = bdh450_ns.class_("BDH450Sensor", cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(BDH450Sensor),
        cv.Required(CONF_CLOCK_PIN): cv.All(pins.internal_gpio_input_pin_schema),
        cv.Required(CONF_DATA_PIN): cv.All(pins.internal_gpio_input_pin_schema),
        cv.Required(CONF_STROBE_PIN): cv.All(pins.internal_gpio_input_pin_schema),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    }
).extend(cv.polling_component_schema("60s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin_clock = await gpio_pin_expression(config[CONF_CLOCK_PIN])
    cg.add(var.set_clk_pin(pin_clock))
    pin_data = await gpio_pin_expression(config[CONF_DATA_PIN])
    cg.add(var.set_dio_pin(pin_data))
    pin_strobe = await gpio_pin_expression(config[CONF_STROBE_PIN])
    cg.add(var.set_dio_pin(pin_strobe))
