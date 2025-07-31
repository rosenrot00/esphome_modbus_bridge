# ESPHome Modbus TCP to RTU Bridge

This ESPHome component implements a transparent Modbus TCP to Modbus RTU bridge that acts as a Modbus RTU master over UART. It allows multiple Modbus TCP clients to communicate with Modbus RTU slaves via RS485 or other UART-compatible hardware.

#### Protocol Overview

The bridge listens on a configurable TCP port (default: 502) and expects standard Modbus TCP frames from clients. Each request is translated into a Modbus RTU frame, transmitted over UART, and the response is converted back into Modbus TCP and returned to the client.

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

#### Features

- Acts as a Modbus RTU master on the UART interface
- Accepts Modbus TCP connections from multiple clients
- Translates TCP frames to RTU and vice versa
- Automatically detects end of RTU frames using UART silence (no byte count parsing required)
- Works with any Modbus function code
- Compatible with Home Assistant and third-party Modbus TCP tools

#### ESPHome Configuration Example

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/rosenrot00/esphome_modbus_bridge
    components: [modbus_bridge]

uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
  stop_bits: 1

modbus_bridge:
  id: mb_bridge
  uart_id: uart_bus
  tcp_port: 502           #tcp port
  tcp_poll_interval: 50   #ms delay between catching new data from tcp
  client_timeout: 60000   #ms timeout tcp clients get disconnected if inactive
  response_timeout: 1200  #ms modbus rtu response timeout

switch:
  - platform: template
    name: "Modbus Bridge Debug"
    id: modbus_debug_switch
    restore_mode: RESTORE_DEFAULT_OFF  ##debug is disabled by default on first boot, and the state is preserved across reboots.
    turn_on_action:
      - lambda: |-
          id(mb_bridge).set_debug(true);
          id(modbus_debug_switch).publish_state(true);
    turn_off_action:
      - lambda: |-
          id(mb_bridge).set_debug(false);
          id(modbus_debug_switch).publish_state(false);
```
#### Proven Compatibility

This bridge has been tested successfully with the [homeassistant-solax-modbus](https://github.com/wills106/homeassistant-solax-modbus) integration.
```
[07:32:40][D][modbus_bridge:104]: TCP->RTU UID: 1, FC: 0x03, LEN: 6
[07:32:40][D][modbus_bridge:109]: RTU send: 01 03 01 02 00 5F A5 CE 
[07:32:40][D][modbus_bridge:150]: RTU recv (195 bytes): 01 03 BE 00 00 00 00 00 03 00 03 00 00 00 00 00 01 00 01 00 02 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 01 00 00 0A C8 07 30 00 00 00 00 00 00 00 00 00 00 00 00 00 1E 00 00 00 00 00 00 07 D0 00 50 03 E8 00 28 00 1E 03 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 F4 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[07:32:40][D][modbus_bridge:167]: RTU->TCP response: 09 BE 00 00 00 C1 01 03 BE 00 00 00 00 00 03 00 03 00 00 00 00 00 01 00 01 00 02 00 00 00 00 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 01 00 00 0A C8 07 30 00 00 00 00 00 00 00 00 00 00 00 00 00 1E 00 00 00 00 00 00 07 D0 00 50 03 E8 00 28 00 1E 03 20 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 F4 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
[07:32:40][D][modbus_bridge:168]: Response time: 106ms
[07:32:40][D][modbus_bridge:104]: TCP->RTU UID: 1, FC: 0x04, LEN: 6
[07:32:40][D][modbus_bridge:109]: RTU send: 01 04 00 00 00 55 30 35 
[07:32:40][D][modbus_bridge:150]: RTU recv (175 bytes): 01 04 AA 09 65 00 0B 01 5E 0F 32 15 DC 00 2D 00 44 13 87 00 31 00 02 06 DF 0E EA 00 1E 10 EE 17 E6 00 02 09 5B 13 87 00 07 00 00 0A A8 00 B6 13 80 00 01 00 18 00 01 00 00 00 00 00 3E 01 37 00 00 00 00 00 25 01 63 00 00 00 1F 01 9A 01 C2 32 00 00 00 00 01 00 00 01 9A 00 0A 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05 50 00 00 00 5B
[07:32:40][D][modbus_bridge:167]: RTU->TCP response: 09 BF 00 00 00 AD 01 04 AA 09 65 00 0B 01 5E 0F 32 15 DC 00 2D 00 44 13 87 00 31 00 02 06 DF 0E EA 00 1E 10 EE 17 E6 00 02 09 5B 13 87 00 07 00 00 0A A8 00 B6 13 80 00 01 00 18 00 01 00 00 00 00 00 3E 01 37 00 00 00 00 00 25 01 63 00 00 00 1F 01 9A 01 C2 32 00 00 00 00 01 00 00 01 9A 00 0A 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 05
[07:32:40][D][modbus_bridge:168]: Response time: 117ms
```
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
- `pymodbus` library  
  Install via: pip install pymodbus
