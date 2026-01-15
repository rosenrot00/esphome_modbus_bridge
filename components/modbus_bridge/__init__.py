import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import automation, pins
from esphome.components import uart
from esphome.const import CONF_ID, CONF_TRIGGER_ID

CONF_TCP_PORT = "tcp_port"
CONF_TCP_POLL_INTERVAL = "tcp_poll_interval"
CONF_TCP_CLIENT_TIMEOUT = "tcp_client_timeout"
CONF_RTU_RESPONSE_TIMEOUT = "rtu_response_timeout"
CONF_TCP_ALLOWED_CLIENTS = "tcp_allowed_clients"
CONF_DE_PIN = "de_pin"
CONF_RE_PIN = "re_pin"
CONF_CRC_BYTES_SWAPPED = "crc_bytes_swapped"
CONF_ENABLED = "enabled"
CONF_UART_WAKE_LOOP_ON_RX = "uart_wake_loop_on_rx"

CONF_ON_COMMAND_SENT = "on_command_sent"
CONF_ON_RTU_SEND = "on_rtu_send"
CONF_ON_RTU_RECEIVE = "on_rtu_receive"
CONF_ON_RTU_TIMEOUT = "on_rtu_timeout"
CONF_ON_TCP_STARTED = "on_tcp_started"
CONF_ON_TCP_STOPPED = "on_tcp_stopped"
CONF_ON_TCP_CLIENTS_CHANGED = "on_tcp_clients_changed"

modbus_bridge_ns = cg.esphome_ns.namespace("modbus_bridge")
ModbusBridgeComponent = modbus_bridge_ns.class_("ModbusBridgeComponent", cg.Component)

BASE_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ModbusBridgeComponent),
        cv.Required("uart_id"): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_DE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_RE_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_TCP_PORT, default=502): cv.port,
        cv.Optional(CONF_TCP_POLL_INTERVAL, default=50): cv.positive_int,
        cv.Optional(CONF_TCP_CLIENT_TIMEOUT, default=60000): cv.positive_int,
        cv.Optional(CONF_RTU_RESPONSE_TIMEOUT, default=3000): cv.positive_int,
        cv.Optional(CONF_TCP_ALLOWED_CLIENTS, default=2): cv.positive_int,
        cv.Optional(CONF_CRC_BYTES_SWAPPED, default=False): cv.boolean,
        cv.Optional(CONF_ENABLED, default=True): cv.boolean,
        cv.Optional(CONF_UART_WAKE_LOOP_ON_RX, default=False): cv.boolean,
        # Expose bridge-global events to YAML automations
        cv.Optional(CONF_ON_COMMAND_SENT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.int_, cg.int_)
                )
            }
        ),
        cv.Optional(CONF_ON_RTU_SEND): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.int_, cg.int_)
                )
            }
        ),
        cv.Optional(CONF_ON_RTU_RECEIVE): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.int_, cg.int_)
                )
            }
        ),
        cv.Optional(CONF_ON_RTU_TIMEOUT): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.int_, cg.int_)
                )
            }
        ),
        cv.Optional(CONF_ON_TCP_STARTED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template()
                )
            }
        ),
        cv.Optional(CONF_ON_TCP_STOPPED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template()
                )
            }
        ),
        cv.Optional(CONF_ON_TCP_CLIENTS_CHANGED): automation.validate_automation(
            {
                cv.GenerateID(CONF_TRIGGER_ID): cv.declare_id(
                    automation.Trigger.template(cg.int_)
                )
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

# Allow a list of bridge definitions under modbus_bridge:
CONFIG_SCHEMA = cv.ensure_list(BASE_SCHEMA)


async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)

        uart_comp = await cg.get_variable(conf["uart_id"])
        # Optional: request low-latency UART RX wake-up (enables USE_UART_WAKE_LOOP_ON_RX)
        if conf.get(CONF_UART_WAKE_LOOP_ON_RX):
            uart.request_wake_loop_on_rx()
        cg.add(var.set_uart_id(uart_comp))
        cg.add(var.set_tcp_port(conf[CONF_TCP_PORT]))
        cg.add(var.set_tcp_poll_interval(conf[CONF_TCP_POLL_INTERVAL]))
        cg.add(var.set_tcp_client_timeout(conf[CONF_TCP_CLIENT_TIMEOUT]))
        cg.add(var.set_rtu_response_timeout(conf[CONF_RTU_RESPONSE_TIMEOUT]))
        cg.add(var.set_tcp_allowed_clients(conf[CONF_TCP_ALLOWED_CLIENTS]))
        cg.add(var.set_crc_bytes_swapped(conf[CONF_CRC_BYTES_SWAPPED]))
        cg.add(var.set_enabled(conf[CONF_ENABLED]))

        # Bind YAML automations → C++ callbacks (function_code, address)
        async def _bind(list_key: str, adder: str):
            if list_key in conf:
                for ac in conf[list_key]:
                    trig = cg.new_Pvariable(ac[CONF_TRIGGER_ID])
                    # C++ lambda triggers the automation with (function_code, address)
                    cb = cg.RawExpression(
                        f"[](int fc, int addr) {{ {trig}->trigger(fc, addr); }}"
                    )
                    cg.add(getattr(var, adder)(cb))
                    await automation.build_automation(
                        trig, [(cg.int_, "function_code"), (cg.int_, "address")], ac
                    )

        await _bind(CONF_ON_COMMAND_SENT, "add_on_command_sent_callback")
        # RTU-centric events
        await _bind(CONF_ON_RTU_SEND, "add_on_command_sent_callback")  # alias: RTU send
        await _bind(CONF_ON_RTU_RECEIVE, "add_on_rtu_receive_callback")
        await _bind(CONF_ON_RTU_TIMEOUT, "add_on_rtu_timeout_callback")

        # Bind simple no-arg TCP server events
        if CONF_ON_TCP_STARTED in conf:
            for ac in conf[CONF_ON_TCP_STARTED]:
                trig = cg.new_Pvariable(ac[CONF_TRIGGER_ID])
                cb = cg.RawExpression(f"[]() {{ {trig}->trigger(); }}")
                cg.add(var.add_on_tcp_started_callback(cb))
                await automation.build_automation(trig, [], ac)

        if CONF_ON_TCP_STOPPED in conf:
            for ac in conf[CONF_ON_TCP_STOPPED]:
                trig = cg.new_Pvariable(ac[CONF_TRIGGER_ID])
                cb = cg.RawExpression(f"[]() {{ {trig}->trigger(); }}")
                cg.add(var.add_on_tcp_stopped_callback(cb))
                await automation.build_automation(trig, [], ac)

        if CONF_ON_TCP_CLIENTS_CHANGED in conf:
            for ac in conf[CONF_ON_TCP_CLIENTS_CHANGED]:
                trig = cg.new_Pvariable(ac[CONF_TRIGGER_ID])
                cb = cg.RawExpression(f"[](int count) {{ {trig}->trigger(count); }}")
                cg.add(var.add_on_tcp_clients_changed_callback(cb))
                await automation.build_automation(trig, [(cg.int_, "count")], ac)

        # optional RS-485 DE and /RE pins
        if CONF_DE_PIN in conf:
            de_pin = await cg.gpio_pin_expression(conf[CONF_DE_PIN])
            cg.add(var.set_de_pin(de_pin))

        if CONF_RE_PIN in conf:
            re_pin = await cg.gpio_pin_expression(conf[CONF_RE_PIN])
            cg.add(var.set_re_pin(re_pin))
