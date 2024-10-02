import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, i2c
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    DEVICE_CLASS_ILLUMINANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_LUX,
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
    sensor.sensor_schema(
        unit_of_measurement=UNIT_LUX,
        accuracy_decimals=3,
        device_class=DEVICE_CLASS_ILLUMINANCE,
        state_class=STATE_CLASS_MEASUREMENT,
    )
    .extend(
        {
            cv.GenerateID(): cv.declare_id(MAX30105Sensor),
            cv.Optional(CONF_MODE, default="low_power"): cv.enum(
                MODE_OPTIONS, lower=True
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
    await sensor.register_sensor(var, config)

    cg.add(var.set_mode(config[CONF_MODE]))