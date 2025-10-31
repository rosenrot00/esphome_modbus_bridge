import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import uart
from esphome.const import CONF_ID
from esphome import pins
from esphome import automation
from esphome.const import CONF_TRIGGER_ID

CONF_TCP_PORT = "tcp_port"
CONF_TCP_POLL_INTERVAL = "tcp_poll_interval"
CONF_TCP_CLIENT_TIMEOUT = "tcp_client_timeout"
CONF_RTU_RESPONSE_TIMEOUT = "rtu_response_timeout"
CONF_TCP_ALLOWED_CLIENTS = "tcp_allowed_clients"
CONF_FLOW_CONTROL_PIN = "flow_control_pin"

CONF_ON_COMMAND_SENT = "on_command_sent"
CONF_ON_ONLINE = "on_online"
CONF_ON_OFFLINE = "on_offline"
CONF_ON_TIMEOUT = "on_timeout"

modbus_bridge_ns = cg.esphome_ns.namespace('modbus_bridge')
ModbusBridgeComponent = modbus_bridge_ns.class_('ModbusBridgeComponent', cg.Component)

BASE_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(ModbusBridgeComponent),
    cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_TCP_PORT, default=502): cv.port,
    cv.Optional(CONF_TCP_POLL_INTERVAL, default=50): cv.positive_int,
    cv.Optional(CONF_TCP_CLIENT_TIMEOUT, default=60000): cv.positive_int,
    cv.Optional(CONF_RTU_RESPONSE_TIMEOUT, default=3000): cv.positive_int,
    cv.Optional(CONF_TCP_ALLOWED_CLIENTS, default=4): cv.positive_int,
    # Expose bridge-global events to YAML automations
    cv.Optional(CONF_ON_COMMAND_SENT): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(automation.Trigger)
    }),
    cv.Optional(CONF_ON_ONLINE): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(automation.Trigger)
    }),
    cv.Optional(CONF_ON_OFFLINE): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(automation.Trigger)
    }),
    cv.Optional(CONF_ON_TIMEOUT): automation.validate_automation({
        cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(automation.Trigger)
    }),
}).extend(cv.COMPONENT_SCHEMA)

# Allow a list of bridge definitions under modbus_bridge:
CONFIG_SCHEMA = cv.ensure_list(BASE_SCHEMA)

async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)

        uart_comp = await cg.get_variable(conf["uart_id"])
        cg.add(var.set_uart_id(uart_comp))
        cg.add(var.set_tcp_port(conf[CONF_TCP_PORT]))
        cg.add(var.set_tcp_poll_interval(conf[CONF_TCP_POLL_INTERVAL]))
        cg.add(var.set_tcp_client_timeout(conf[CONF_TCP_CLIENT_TIMEOUT]))
        cg.add(var.set_rtu_response_timeout(conf[CONF_RTU_RESPONSE_TIMEOUT]))
        cg.add(var.set_tcp_allowed_clients(conf[CONF_TCP_ALLOWED_CLIENTS]))

        # Bind YAML automations → C++ callbacks (function_code, address)
        async def _bind(list_key: str, adder: str):
            if list_key in conf:
                for ac in conf[list_key]:
                    trig = cg.new_Pvariable(ac[CONF_TRIGGER_ID])
                    # C++ lambda triggers the automation with (function_code, address)
                    cb = cg.RawExpression(f"[](int fc, int addr) {{ {trig}.trigger(fc, addr); }}")
                    cg.add(getattr(var, adder)(cb))
                    await automation.build_automation(trig, [(cg.int_, "function_code"), (cg.int_, "address")], ac)

        await _bind(CONF_ON_COMMAND_SENT, "add_on_command_sent_callback")
        await _bind(CONF_ON_ONLINE,       "add_on_online_callback")
        await _bind(CONF_ON_OFFLINE,      "add_on_offline_callback")
        await _bind(CONF_ON_TIMEOUT,      "add_on_timeout_callback")

        # optional RS-485 DE/RE flow control pin
        if CONF_FLOW_CONTROL_PIN in conf:
            pin = await cg.gpio_pin_expression(conf[CONF_FLOW_CONTROL_PIN])
            cg.add(var.set_flow_control_pin(pin))
