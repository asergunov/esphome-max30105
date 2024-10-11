import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
    CONF_INFRARED,
    CONF_GREEN,
    CONF_RED
)

CODEOWNERS = ["@asergunov"]
DEPENDENCIES = ["i2c"]

max30105_ns = cg.esphome_ns.namespace("max30105")
MAX30105Sensor = max30105_ns.class_(
    "MAX30105Sensor", sensor.Sensor, cg.PollingComponent, i2c.I2CDevice
)

MAX30105Mode = max30105_ns.enum("MAX30105Mode")
MODE_OPTIONS = {
    # "auto": MAX30105Mode.MAX30105_MODE_AUTO,
    # "low_power": MAX30105Mode.MAX30105_MODE_LOW_POWER,
    # "continuous": MAX30105Mode.MAX30105_MODE_CONTINUOUS,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MAX30105Sensor),
            cv.Optional(CONF_INFRARED): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_GREEN): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_RED): sensor.sensor_schema(
                unit_of_measurement=UNIT_LUX,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_ILLUMINANCE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),

        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x57))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if red := config.get(CONF_RED):
        sens = await sensor.new_sensor(red)
        cg.add(var.set_red_sensor(sens))

    if green := config.get(CONF_GREEN):
        sens = await sensor.new_sensor(green)
        cg.add(var.set_green_sensor(sens))

    if ir := config.get(CONF_INFRARED):
        sens = await sensor.new_sensor(ir)
        cg.add(var.set_ir_sensor(sens))
