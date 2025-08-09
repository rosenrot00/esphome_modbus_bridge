import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import uart
from esphome.const import CONF_ID

CONF_TCP_PORT = "tcp_port"
CONF_TCP_POLL_INTERVAL = "tcp_poll_interval"
CONF_TCP_CLIENT_TIMEOUT = "tcp_client_timeout"
CONF_RTU_RESPONSE_TIMEOUT = "rtu_response_timeout"
CONF_TCP_ALLOWED_CLIENTS = "tcp_allowed_clients"

modbus_bridge_ns = cg.esphome_ns.namespace('modbus_bridge')
ModbusBridgeComponent = modbus_bridge_ns.class_('ModbusBridgeComponent', cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusBridgeComponent),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_TCP_PORT, default=502): cv.port,
    cv.Optional(CONF_TCP_POLL_INTERVAL, default=50): cv.positive_int,
    cv.Optional(CONF_TCP_CLIENT_TIMEOUT, default=60000): cv.positive_int,
    cv.Optional(CONF_RTU_RESPONSE_TIMEOUT, default=3000): cv.positive_int,
    cv.Optional(CONF_TCP_ALLOWED_CLIENTS, default=4): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    uart_comp = await cg.get_variable(config["uart_id"])
    cg.add(var.set_uart_id(uart_comp))
    cg.add(var.set_tcp_port(config[CONF_TCP_PORT]))
    cg.add(var.set_tcp_poll_interval(config[CONF_TCP_POLL_INTERVAL]))
    cg.add(var.set_tcp_client_timeout(config[CONF_TCP_CLIENT_TIMEOUT]))
    cg.add(var.set_rtu_response_timeout(config[CONF_RTU_RESPONSE_TIMEOUT]))
    cg.add(var.set_tcp_allowed_clients(config[CONF_TCP_ALLOWED_CLIENTS]))
