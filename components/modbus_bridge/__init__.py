import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import uart
from esphome.const import CONF_ID, CONF_DEBUG

CONF_TCP_PORT = "tcp_port"
CONF_TCP_POLL_INTERVAL = "tcp_poll_interval"

modbus_bridge_ns = cg.esphome_ns.namespace('modbus_bridge')
ModbusBridgeComponent = modbus_bridge_ns.class_('ModbusBridgeComponent', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusBridgeComponent),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_TCP_PORT, default=502): cv.port,
    cv.Optional(CONF_TCP_POLL_INTERVAL, default=10): cv.positive_int,
    cv.Optional(CONF_DEBUG, default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config["uart_id"])
    cg.add(var.set_uart_id(uart_comp))
    cg.add(var.set_tcp_port(config[CONF_TCP_PORT]))
    cg.add(var.set_debug(config[CONF_DEBUG]))
    cg.add(var.set_tcp_poll_interval(config[CONF_TCP_POLL_INTERVAL]))
