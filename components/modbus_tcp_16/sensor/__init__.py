import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_EMPTY,
    STATE_CLASS_MEASUREMENT,
    UNIT_EMPTY,
)

# Dependencies - use ESPHome's built-in networking
DEPENDENCIES = ["wifi", "network", "socket"]
CODEOWNERS = ["@Gucioo"]

# Namespace and class definitions
modbus_tcp_16_ns = cg.esphome_ns.namespace("modbus_tcp_16")
ModbusTCP16 = modbus_tcp_16_ns.class_("ModbusTCP16", cg.Component, sensor.Sensor)

# Configuration constants - define our own since they're not in esphome.const
CONF_HOST = "host"
CONF_PORT = "port"
CONF_SLAVE_ID = "slave_id"
CONF_REGISTER_ADDRESS = "register_address"
CONF_REGISTER_COUNT = "register_count"

CONFIG_SCHEMA = (
    sensor.sensor_schema(
        ModbusTCP16,
        device_class=DEVICE_CLASS_EMPTY,
        state_class=STATE_CLASS_MEASUREMENT,
        unit_of_measurement=UNIT_EMPTY,
    )
    .extend(
        {
            cv.Required(CONF_HOST): cv.string,
            cv.Optional(CONF_PORT, default=502): cv.port,
            cv.Optional(CONF_SLAVE_ID, default=1): cv.uint8_t,
            cv.Required(CONF_REGISTER_ADDRESS): cv.uint16_t,
            cv.Optional(CONF_REGISTER_COUNT, default=1): cv.uint16_t,
            cv.Optional(CONF_UPDATE_INTERVAL, default="60s"): cv.update_interval,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

async def to_code(config):
    var = await sensor.new_sensor(config)
    await cg.register_component(var, config)

    cg.add(var.set_host(config[CONF_HOST]))
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_slave_id(config[CONF_SLAVE_ID]))
    cg.add(var.set_register_address(config[CONF_REGISTER_ADDRESS]))
    cg.add(var.set_register_count(config[CONF_REGISTER_COUNT]))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL].total_milliseconds))

    # No external libraries needed - ESPHome has built-in socket support
