import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import uart
from esphome.const import CONF_ID, CONF_DEBUG

modbus_bridge_ns = cg.esphome_ns.namespace('modbus_bridge')
ModbusBridgeComponent = modbus_bridge_ns.class_('ModbusBridgeComponent', cg.PollingComponent)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusBridgeComponent),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Optional("tcp_port", default=502): cv.port,
    cv.Optional("debug", default=False): cv.boolean,
}).extend(cv.polling_component_schema("60s"))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    uart_comp = await cg.get_variable(config["uart_id"])
    cg.add(var.set_uart_id(uart_comp))
    cg.add(var.set_tcp_port(config["tcp_port"]))
    cg.add(var.set_debug(config["debug"]))
