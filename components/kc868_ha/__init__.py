import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import uart

DEPENDENCIES = ["uart"]
AUTO_LOAD = ['binary_sensor', 'switch']
MULTI_CONF = False

KC868_HA_ID = 'kc868_ha_id'
CONF_TX_QUIET_TIME = 'tx_quiet_time'
CONF_TX_GUARD_TIME = 'tx_guard_time'

kc868_ha_ns = cg.esphome_ns.namespace('kc868_ha')

Kc868HaComponent = kc868_ha_ns.class_('KC868HaComponent', cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Kc868HaComponent),
    cv.Optional(CONF_TX_QUIET_TIME, default="100ms"): cv.positive_time_period_milliseconds,
    cv.Optional(CONF_TX_GUARD_TIME, default="100ms"): cv.positive_time_period_milliseconds,
}).extend(uart.UART_DEVICE_SCHEMA).extend(cv.COMPONENT_SCHEMA)

def to_code(config):
    u = yield cg.get_variable(config["uart_id"])
    var = cg.new_Pvariable(config[CONF_ID], u)
    yield cg.register_component(var, config)
    yield uart.register_uart_device(var, config)
    cg.add(var.set_tx_quiet_time(config[CONF_TX_QUIET_TIME].total_milliseconds))
    cg.add(var.set_tx_guard_time(config[CONF_TX_GUARD_TIME].total_milliseconds))
