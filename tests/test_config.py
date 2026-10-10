"""Run with a Python environment containing ESPHome 2026.9.1 or newer."""

from pathlib import Path
import re
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BASE = f"""esphome:
  name: bridge-config-test
esp32:
  board: esp32dev
  framework:
    type: esp-idf
logger:
wifi:
  ssid: test-network
  password: test-password
external_components:
  - source:
      type: local
      path: {ROOT / 'components'}
"""
UART = """uart:
  id: uart_bus
  tx_pin: GPIO17
  rx_pin: GPIO16
  baud_rate: 9600
"""
HUB = """modbus:
  id: shared_hub
  uart_id: uart_bus
  role: client
  turnaround_time: 0ms
"""


class ConfigTests(unittest.TestCase):
    def check_config(self, contents, error=None, generate=False):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "test.yaml"
            path.write_text(BASE + contents)
            command = [sys.executable, "-m", "esphome"]
            command += ["compile", str(path), "--only-generate"] if generate else ["config", str(path)]
            result = subprocess.run(command, capture_output=True, text=True)
            output = result.stdout + result.stderr
            if error:
                self.assertNotEqual(result.returncode, 0, output)
                self.assertIn(error, output)
            else:
                self.assertEqual(result.returncode, 0, output)
                if generate:
                    return (Path(directory) / ".esphome/build/bridge-config-test/src/esphome/core/defines.h").read_text()

    def test_direct_backend(self):
        defines = self.check_config(UART + "modbus_bridge:\n  uart_id: uart_bus\n", generate=True)
        self.assertNotIn("USE_MODBUS_BRIDGE_HUB", defines)

    def test_shared_hub_readme_example(self):
        readme = (ROOT / "README.md").read_text()
        section = readme.split("##### Shared Modbus Hub", 1)[1]
        example = re.search(r"```yaml\n(.*?)```", section, re.S).group(1)
        defines = self.check_config(example, generate=True)
        self.assertIn("USE_MODBUS_BRIDGE_HUB", defines)

    def test_mixed_backends(self):
        self.check_config(HUB + """modbus_bridge:
  - id: direct_bridge
    uart_id: second_uart
    tcp_port: 503
  - id: shared_bridge
    modbus_id: shared_hub
uart:
  - id: uart_bus
    tx_pin: GPIO17
    rx_pin: GPIO16
    baud_rate: 9600
  - id: second_uart
    tx_pin: GPIO25
    rx_pin: GPIO26
    baud_rate: 9600
""", generate=True)

    def test_backend_is_required(self):
        self.check_config(UART + "modbus_bridge:\n  id: bridge\n", error="exactly one")

    def test_backends_are_mutually_exclusive(self):
        self.check_config(UART + HUB + "modbus_bridge:\n  uart_id: uart_bus\n  modbus_id: shared_hub\n", error="Cannot specify more than one")

    def test_direct_only_options_rejected_with_hub(self):
        for key, value in (("rtu_response_timeout", "1000"), ("crc_bytes_swapped", "false"),
                           ("de_pin", "GPIO18"), ("re_pin", "GPIO19")):
            with self.subTest(option=key):
                self.check_config(UART + HUB + f"modbus_bridge:\n  modbus_id: shared_hub\n  {key}: {value}\n",
                                  error=f"{key} is only available with uart_id")

    def test_server_hub_rejected(self):
        self.check_config(UART + HUB.replace("role: client", "role: server").replace("  turnaround_time: 0ms\n", "") +
                          "modbus_bridge:\n  modbus_id: shared_hub\n", error="ModbusClientHub")


if __name__ == "__main__":
    unittest.main()
