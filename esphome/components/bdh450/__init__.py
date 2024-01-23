import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import sensor, binary_sensor, text_sensor
from esphome.const import (
    CONF_ID,
    CONF_CLOCK_PIN,
    CONF_DATA_PIN,
    CONF_HUMIDITY,
    CONF_POWER,
    CONF_FAN_MODE,
    DEVICE_CLASS_HUMIDITY,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

CONF_TANK_STATUS = "tank_status"
CONF_DEFROSTING = "defrosting"
CONF_STROBE_PIN = "strobe_pin"

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
        cv.Optional(CONF_POWER): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_DEFROSTING): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_TANK_STATUS): binary_sensor.binary_sensor_schema(),
        cv.Optional(CONF_FAN_MODE): text_sensor.text_sensor_schema(text_sensor.TextSensor)
    }
).extend(cv.polling_component_schema("2s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin_clock = await gpio_pin_expression(config[CONF_CLOCK_PIN])
    cg.add(var.set_clk_pin(pin_clock))
    pin_data = await gpio_pin_expression(config[CONF_DATA_PIN])
    cg.add(var.set_dio_pin(pin_data))
    pin_strobe = await gpio_pin_expression(config[CONF_STROBE_PIN])
    cg.add(var.set_dio_pin(pin_strobe))


    if CONF_POWER in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))

    if CONF_TANK_STATUS in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_TANK_STATUS])
        cg.add(var.set_tank_sensor(sens))

    if CONF_DEFROSTING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DEFROSTING])
        cg.add(var.set_defrost_sensor(sens))

    if CONF_FAN_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAN_MODE])
        cg.add(var.set_fan_mode_sensor(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity_sensor(sens))
