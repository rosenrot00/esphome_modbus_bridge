# ESPHome (ESP8266/ESP32) Modbus TCP to RTU Bridge

This ESPHome component provides a transparent Modbus TCP-to-RTU bridge, acting as a Modbus RTU master over UART on both ESP8266 and ESP32 platforms. It allows multiple Modbus TCP clients to communicate with Modbus RTU slaves via RS485 or other UART-compatible hardware.

| Version   | Changes                                                                           |
|-----------|-----------------------------------------------------------------------------------|
| 2025.12.3 | Added `uart_wake_loop_on_rx` to enable ESPHome’s low-latency UART flag            |
| 2025.12.2 | Optimizations to recover after IP loss and tighten RTU frame detection            |
| 2025.12.1 | For more compatibility a `crc_bytes_swapped` option was added                     |
| 2025.11.1 | `enabled` was added to allow changing the bridges state during runtime            |
| 2025.10.3 | Added ESPHome automations for tcp and rtu activities                              |
| 2025.10.2 | Introduced T1.5 waiting time for better modbus rtu frame detection on lower bauds |
| 2025.10.1 | Implemented support for multiple bridges to be used with multiple UART interfaces |
| 2025.09.1 | Added configurable `flow_control_pin` with inverted option                        |
| 2025.08.2 | Improved RTU response handling (silence-based end detection)                      |
| 2025.08.1 | Added support for multiple concurrent TCP clients with preemption logic           |
| 2025.07.1 | Initial public README and Python `modbus_rw.py` tool                              |

#### Features

The bridge listens on a configurable TCP port (default: 502) and expects standard Modbus TCP frames from clients. Each request is translated into a Modbus RTU frame, transmitted over UART, and the response is converted back into Modbus TCP and returned to the client.

- Acts as a Modbus RTU master on UART
- Multiple concurrent Modbus TCP clients (slot‑limited)
- TCP↔RTU translation both ways
- RTU end‑of‑frame via UART silence (no byte count needed)
- Works with all Modbus function codes
- Optional same‑IP preemption when slots are full
- Compatible with Home Assistant and third‑party Modbus TCP tools

#### Proven Compatibility
- [nilan-cts600-homeassistant](https://github.com/frodef/nilan-cts600-homeassistant) thanks to @RichardIstSauer
- [ha-solarman](https://github.com/davidrapan/ha-solarman) thanks to @davidrapan
- [Marstek Venus Battery](https://github.com/ViperRNMC/marstek_venus_modbus) thanks to @ebbenberg
- [homeassistant-solax-modbus](https://github.com/wills106/homeassistant-solax-modbus)

#### Hardware Setup
The following diagram shows how an ESP32 is connected to an RS485 transceiver (e.g., MAX3485, SP3485, SN65HVD…) before the RS485 differential lines are attached to a Modbus bus.
```
             +--------------------+         +---------------------------+
             |        ESP32       |         |      RS485 Transceiver    |
             |       ESP8266      |         |   (e.g. MAX3485/SP3485)   |
             +--------------------+         +---------------------------+
             | GPIO TX (UART TX)  |-------->| DI        (Data In)       |
             | GPIO RX (UART RX)  |<--------| RO        (Receiver Out)  |
             | GPIO DE (Driver En)|-------->| DE        (Driver Enable) |
             | GPIO RE (Recv En)  |-------->| /RE       (Recv Enable)   |
             | GND                |---------| GND                       |
             +--------------------+         +------------+--------------+
                                                     |
                                                     |
                                                     |   RS485 differential pair
                                                     |   (before connecting to Modbus)
                                                     |
                                              +------+------+ 
                                              |   A   |   B |
                                              +------+------+
```

#### ESPHome Configuration Example

```yaml
esphome:
  name: modbus-bridge
  friendly_name: Modbus TCP-to-RTU bridge

  # Run on boot: publish whether the bridge is enabled
  on_boot:
    priority: 600
    then:
      - lambda: |-
          id(mb_bridge_enabled).publish_state(id(mb_bridge).is_enabled());

esp32:
  board: esp32dev
  framework:
    type: esp-idf                    # ESP-IDF recommended
    #type: arduino                   # Arduino also works

# Enable logging over UART
logger:

# Enable Home Assistant API
api:

# Enable OTA updates
ota:
  platform: esphome
  password: !secret ota_password      # https://esphome.io/guides/security_best_practices/#using-secretsyaml

wifi:
  ssid: !secret wifi_ssid             # https://esphome.io/guides/security_best_practices/#using-secretsyaml
  password: !secret wifi_password     # https://esphome.io/guides/security_best_practices/#using-secretsyaml
  # min_auth_mode: WPA3               # Optional: Default is WPA2 on ESP32
  # domain: .lan                      # Optional: Default is local 

  # Fallback hotspot if WiFi fails
  ap:
    ssid: "Modbus TCP-to-RTU bridge Hotspot"
    password: !secret ap_password     # https://esphome.io/guides/security_best_practices/#using-secretsyaml

captive_portal:

external_components:
  - source:
      type: git
      url: https://github.com/rosenrot00/esphome_modbus_bridge
    components: [modbus_bridge]

# UART hardware configuration: Modbus RTU (RS-485)
uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  # stop_bits: 1                 # Optional: Default is 1
  # parity: NONE                 # Optional: Default is NONE
  rx_buffer_size: 256            # minimum 256 recommended; increase for long RTU responses

# Modbus bridge configuration: TCP server <-> UART RTU translator
modbus_bridge:
  id: mb_bridge
  uart_id: uart_bus
  tcp_port: 502                  # TCP port to listen on
  rtu_response_timeout: 3000     # ms, internally clamped to >=10 ms
  # tcp_client_timeout: 60000    # ms of inactivity before client is disconnected
  # tcp_allowed_clients: 2       # number of simultaneous TCP clients (min 1)
  # tcp_poll_interval: 50        # ms between TCP polls
  # flow_control_pin: GPIO18     # Optional: RS-485 DE/RE pin
  # crc_bytes_swapped: false     # allows to swap CRC byte order LO/HI -> HI/LO
  # enabled: true                # allows to enable or disable during runtime
  # uart_wake_loop_on_rx: true   # enable ESPHome's UART low latency setting (effects not yet tested)

  # Event: triggered whenever number of TCP clients changes
  on_tcp_clients_changed:
    then:
      - lambda: |-
          id(tcp_clients) = count;
      - logger.log:
          format: "TCP clients connected: %d"
          args: ['count']

  # Other available events (use similarly):
  # on_rtu_send:       # (function_code, address) – triggered for every RTU command sent
  # on_rtu_receive:    # (function_code, address) – triggered for every valid RTU response
  # on_rtu_timeout:    # (function_code, address) – triggered for RTU timeouts
  # on_tcp_started:    # () – triggered when TCP server successfully starts
  # on_tcp_stopped:    # () – triggered when TCP server stops or IP is lost

# Output pin for status LED
output:
  - platform: gpio
    id: output_led_status
    pin: GPIO2

# Binary LED light entity
light:
  - platform: binary
    id: led_status
    name: "Status LED"
    output: output_led_status

# Global variable to store connected TCP client count
globals:
  - id: tcp_clients
    type: int
    restore_value: no
    initial_value: '0'

# Every 3 seconds, blink the LED N times (N = connected TCP clients)
interval:
  - interval: 3s
    then:
      - if:
          condition:
            lambda: 'return id(tcp_clients) > 0;'
          then:
            - repeat:
                count: !lambda 'return id(tcp_clients);'
                then:
                  - light.turn_on: led_status
                  - delay: 100ms
                  - light.turn_off: led_status
                  - delay: 100ms  # short pause between blinks

switch:
  # Switch: enable/disable verbose Modbus debugging
  - platform: template
    name: "Modbus Bridge Debug"
    id: modbus_debug_switch
    restore_mode: RESTORE_DEFAULT_OFF
    turn_on_action:
      - lambda: |-
          id(mb_bridge).set_debug(true);
          id(modbus_debug_switch).publish_state(true);
    turn_off_action:
      - lambda: |-
          id(mb_bridge).set_debug(false);
          id(modbus_debug_switch).publish_state(false);

  # Switch: enable/disable the Modbus bridge itself
  - platform: template
    id: mb_bridge_enabled
    name: "Modbus Bridge Enabled"
    restore_mode: "ALWAYS_ON"
    optimistic: true
    turn_on_action:
      - lambda: |-
          id(mb_bridge).set_enabled(true);
    turn_off_action:
      - lambda: |-
          id(mb_bridge).set_enabled(false);
```

#### Modbus TCP Request Format

Each Modbus TCP request must follow this format:

- **Transaction ID**: 2 bytes (arbitrary, echoed back)
- **Protocol ID**: 2 bytes (must be 0)
- **Length**: 2 bytes (number of following bytes, typically `unit id` + `PDU`)
- **Unit ID**: 1 byte (RTU slave address)
- **PDU**: n bytes (Function code and data)

Example (read holding registers, unit ID 1, starting at 0x0000, count 1):
```
00 01   - Transaction ID
00 00   - Protocol ID
00 06   - Length
01      - Unit ID (RTU address)
03      - Function code (Read Holding Registers)
00 00   - Start address high/low
00 01   - Register count high/low
```
The response will match the Modbus TCP format and contain the same transaction ID.

## modbus_rw.py – Modbus TCP Register Read/Write Tool

`modbus_rw.py` is a simple command-line utility for reading and writing Modbus TCP registers using the `pymodbus` library.  
It supports reading Holding Registers (Function Code 0x03), Input Registers (0x04), and writing a single Holding Register (0x06).  
This tool is useful for testing, diagnostics, or integrating Modbus-capable devices in a network environment.
#### Arguments
```
--host         Modbus TCP server IP address (required)
--port         Modbus TCP port (default: 502)
--unit         Modbus unit ID / slave ID (default: 1)
--register     Register address to read/write (decimal or hex, e.g. 0x10) (required)
--count        Number of registers to read (default: 1)
--value        Value to write to register (used for write operation)
--read         Read Holding Registers (Function Code 0x03)
--read_input   Read Input Registers (Function Code 0x04)
```
#### Examples

- Read Holding Registers (FC 0x03)
```
python modbus_rw.py --host 192.168.0.10 --register 0x0010 --count 2 --read
```
- Read Input Registers (FC 0x04)
```
python modbus_rw.py --host 192.168.0.10 --register 0x0010 --count 2 --read_input
```
- Write a Single Holding Register (FC 0x06)
```
python modbus_rw.py --host 192.168.0.10 --register 0x0010 --value 0x1234
```
#### Requirements
- Python 3.x  
- pymodbus ≤3.9.x library (let me know if you need it compatible with >3.10)
- Install via: `pip install "pymodbus<3.10"`
