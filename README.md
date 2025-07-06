# ESPHome Modbus TCP to RTU Bridge

This ESPHome component implements a Modbus TCP to Modbus RTU bridge that acts as a Modbus RTU master over UART. It allows multiple Modbus TCP clients to communicate with Modbus RTU slaves via RS485 or other UART-compatible hardware.

## Protocol Overview

The bridge listens on a configurable TCP port (default: 502) and expects standard Modbus TCP frames from clients. Each request is translated into a Modbus RTU frame, transmitted over UART, and the response is converted back into Modbus TCP and returned to the client.

### Modbus TCP Request Format

Each Modbus TCP request must follow this format:

- **Transaction ID**: 2 bytes (arbitrary, echoed back)
- **Protocol ID**: 2 bytes (must be 0)
- **Length**: 2 bytes (number of following bytes, typically `unit id` + `PDU`)
- **Unit ID**: 1 byte (RTU slave address)
- **PDU**: n bytes (Function code and data)

Example (read holding registers, unit ID 1, starting at 0x0000, count 1):

00 01   - Transaction ID
00 00   - Protocol ID
00 06   - Length
01      - Unit ID (RTU address)
03      - Function code (Read Holding Registers)
00 00   - Start address high/low
00 01   - Register count high/low

The response will match the Modbus TCP format and contain the same transaction ID.

## Features

- Acts as a Modbus RTU master on the UART interface
- Accepts Modbus TCP connections from multiple clients
- Translates TCP frames to RTU and vice versa
- Automatically detects end of RTU frames using UART silence (no byte count parsing required)
- Works with any Modbus function code
- Compatible with Home Assistant and third-party Modbus TCP tools

## ESPHome Configuration Example

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
  uart_id: uart_bus
  tcp_port: 502
  debug: true
```
