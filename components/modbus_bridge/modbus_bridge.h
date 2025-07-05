// modbus_bridge.h – updated for timing support and long response warning
#pragma once
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include <vector>

namespace esphome {
namespace modbus_bridge {

struct TCPClient {
  int fd = -1;
};

struct PendingRequest {
  int client_fd;
  uint8_t header[7];
  std::vector<uint8_t> response;
  bool active = false;
  uint32_t start_time = 0;
};

class ModbusBridgeComponent : public PollingComponent {
 public:
  ModbusBridgeComponent();

  void set_uart_id(uart::UARTComponent *uart) { uart_ = uart; }
  void set_tcp_port(uint16_t port) { tcp_port_ = port; }
  void set_debug(bool debug) { debug_ = debug; }

  void setup() override;
  void loop() override;
  void update() override;

 protected:
  uart::UARTComponent *uart_;
  int sock_{-1};
  TCPClient clients_[4];
  PendingRequest pending_request_;
  uint16_t tcp_port_{502};
  bool debug_{false};

  void append_crc(std::vector<uint8_t> &data);
  void initialize_tcp_server_();
  void poll_uart_response_();
};

}  // namespace modbus_bridge
}  // namespace esphome
