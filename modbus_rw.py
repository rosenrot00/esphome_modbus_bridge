import argparse
import logging

from pymodbus.client import ModbusTcpClient, ModbusSerialClient

logging.basicConfig(
    format="%(asctime)s %(levelname)s: %(message)s",
    level=logging.DEBUG,
)

def build_client(args):
    if args.mode == "tcp":
        return ModbusTcpClient(
            host=args.host,
            port=args.tcp_port,
            timeout=args.timeout,
        )
    else:
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
    parser = argparse.ArgumentParser(description="Modbus TCP / RTU Register Read/Write")

    parser.add_argument("--mode", choices=["tcp", "rtu"], required=True)

    # TCP
    parser.add_argument("--host", help="Modbus TCP Host")
    parser.add_argument("--tcp-port", type=int, default=502)

    # RTU / USB
    parser.add_argument("--serial-port", help="Serial port (COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument("--parity", default="N", choices=["N", "E", "O"])
    parser.add_argument("--stopbits", type=int, default=1, choices=[1, 2])
    parser.add_argument("--bytesize", type=int, default=8, choices=[7, 8])

    # Common
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--unit", type=int, default=1)
    parser.add_argument("--register", required=True, type=lambda x: int(x, 0))
    parser.add_argument("--value", type=lambda x: int(x, 0))
    parser.add_argument("--count", type=int, default=1)
    parser.add_argument("--read", action="store_true")
    parser.add_argument("--read_input", action="store_true")

    args = parser.parse_args()

    # Minimal validation
    if args.mode == "tcp" and not args.host:
        parser.error("--host required for tcp mode")
    if args.mode == "rtu" and not args.serial_port:
        parser.error("--serial-port required for rtu mode")

    client = build_client(args)

    if not client.connect():
        print("Could not connect")
        return

    try:
        if args.value is not None:
            result = client.write_register(
                address=args.register,
                value=args.value,
                slave=args.unit,
            )
            logging.debug(f"Write response: {result}")

        else:
            if args.read_input:
                result = client.read_input_registers(
                    address=args.register,
                    count=args.count,
                    slave=args.unit,
                )
            else:
                result = client.read_holding_registers(
                    address=args.register,
                    count=args.count,
                    slave=args.unit,
                )

            if result.isError():
                print(result)
            else:
                byte_data = b"".join(
                    r.to_bytes(2, "big") for r in result.registers
                )
                print(" ".join(f"{b:02X}" for b in byte_data))

    finally:
        client.close()

if __name__ == "__main__":
    main()
