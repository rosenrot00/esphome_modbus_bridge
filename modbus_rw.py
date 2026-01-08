import argparse
import logging

from pymodbus.client import ModbusTcpClient, ModbusSerialClient

logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    level=logging.DEBUG,
)

def build_client(args):
    # Implicit selection:
    # - TCP if --host is provided
    # - RTU if --serial-port is provided
    if args.host:
        return ModbusTcpClient(host=args.host, port=args.tcp_port, timeout=args.timeout)
    return ModbusSerialClient(
        port=args.serial_port,
        framer="rtu",
        baudrate=args.baudrate,
        parity=args.parity,
        stopbits=args.stopbits,
        bytesize=args.bytesize,
        timeout=args.timeout,
    )

def main():
    parser = argparse.ArgumentParser(description="Modbus TCP or RTU Register Read/Write (implicit mode)")

    # Implicit mode inputs
    parser.add_argument("--host", help="Modbus TCP Host (if set → TCP mode)")
    parser.add_argument("--tcp-port", type=int, default=502, help="Modbus TCP Port (default: 502)")

    parser.add_argument("--serial-port", help="Serial port (if set → RTU mode), e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=9600, help="RTU baudrate (default: 9600)")
    parser.add_argument("--parity", default="N", choices=["N", "E", "O"], help="RTU parity (default: N)")
    parser.add_argument("--stopbits", type=int, default=1, choices=[1, 2], help="RTU stopbits (default: 1)")
    parser.add_argument("--bytesize", type=int, default=8, choices=[7, 8], help="RTU bytesize (default: 8)")

    # Common
    parser.add_argument("--timeout", type=float, default=1.0, help="Timeout seconds (default: 1.0)")
    parser.add_argument("--unit", default=1, type=int, help="Modbus Unit ID (default: 1)")
    parser.add_argument("--register", required=True, type=lambda x: int(x, 0), help="Register address (decimal or hex)")
    parser.add_argument("--value", type=lambda x: int(x, 0), help="Value to write (decimal or hex)")
    parser.add_argument("--count", type=int, default=1, help="Number of registers to read")
    parser.add_argument("--read", action="store_true", help="Read Holding Registers (FC 0x03)")
    parser.add_argument("--read_input", action="store_true", help="Read Input Registers (FC 0x04)")

    args = parser.parse_args()

    # Validation of implicit mode selection
    if bool(args.host) == bool(args.serial_port):
        parser.error("Specify exactly one of --host (TCP) or --serial-port (RTU).")

    if args.value is None and not (args.read or args.read_input):
        parser.error("Specify either --value (write) or --read / --read_input (read).")

    client = build_client(args)

    if not client.connect():
        if args.host:
            print(f"Could not connect to Modbus TCP server at {args.host}:{args.tcp_port}")
        else:
            print(f"Could not open serial port {args.serial_port}")
        return

    try:
        if args.value is not None:
            print(f"Writing Register (FC 0x06), Addr={args.register}, Value={args.value}")
            result = client.write_register(address=args.register, value=args.value, slave=args.unit)
            logging.debug(f"Write response: {result}")

        else:
            fc_str = "Input Registers (FC 0x04)" if args.read_input else "Holding Registers (FC 0x03)"
            print(f"Reading {fc_str}, Addr={args.register}, Count={args.count}")

            if args.read_input:
                result = client.read_input_registers(address=args.register, count=args.count, slave=args.unit)
            else:
                result = client.read_holding_registers(address=args.register, count=args.count, slave=args.unit)

            if result.isError():
                print(f"Error reading register: {result}")
            else:
                logging.debug(f"Raw register data: {result.registers}")
                byte_data = b"".join(reg.to_bytes(2, byteorder="big") for reg in result.registers)
                raw_hex = " ".join(f"{byte:02X}" for byte in byte_data)
                print(f"Data ({args.count} registers):\n{raw_hex}")

    finally:
        client.close()

if __name__ == "__main__":
    main()
