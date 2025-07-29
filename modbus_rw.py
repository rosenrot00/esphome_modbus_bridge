import argparse
import logging
from pymodbus.client import ModbusTcpClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder

logging.basicConfig(format='%(asctime)s %(levelname)s: %(message)s', level=logging.DEBUG)


def main():
    parser = argparse.ArgumentParser(description="Modbus TCP Register Read/Write")

    parser.add_argument('--host', required=True, help='Modbus TCP Host IP')
    parser.add_argument('--port', default=502, type=int, help='Modbus TCP Port (default: 502)')
    parser.add_argument('--unit', default=1, type=int, help='Modbus Unit ID (default: 1)')
    parser.add_argument('--register', required=True, type=lambda x: int(x, 0), help='Register address (decimal or hex)')
    parser.add_argument('--value', type=lambda x: int(x, 0), help='Value to write (decimal or hex)')
    parser.add_argument('--count', type=int, default=1, help='Number of registers to read')
    parser.add_argument('--read', action='store_true', help='Read Holding Registers (FC 0x03)')
    parser.add_argument('--read_input', action='store_true', help='Read Input Registers (FC 0x04)')

    args = parser.parse_args()

    client = ModbusTcpClient(args.host, port=args.port)

    if not client.connect():
        print(f"Could not connect to Modbus TCP server at {args.host}:{args.port}")
        return

    if args.value is not None:
        print(f"Writing Register (FC 0x06), Addr={args.register}, Value={args.value}")
        result = client.write_register(address=args.register, value=args.value, slave=args.unit)
        logging.debug(f"Write response: {result}")
    elif args.read or args.read_input:
        fc = 4 if args.read_input else 3
        fc_str = 'Input Registers (FC 0x04)' if args.read_input else 'Holding Registers (FC 0x03)'
        print(f"Reading {fc_str}, Addr={args.register}, Count={args.count}")

        if args.read_input:
            result = client.read_input_registers(address=args.register, count=args.count, slave=args.unit)
        else:
            result = client.read_holding_registers(address=args.register, count=args.count, slave=args.unit)

        if result.isError():
            print(f"Error reading register: {result}")
        else:
            logging.debug(f"Raw register data: {result.registers}")
            decoder = BinaryPayloadDecoder.fromRegisters(result.registers, byteorder=Endian.BIG)
            raw_hex = ' '.join(f'{byte:02X}' for byte in decoder.decode_string(len(result.registers) * 2))
            print(f"Data ({args.count} registers):\n{raw_hex}")

    client.close()


if __name__ == "__main__":
    main()
